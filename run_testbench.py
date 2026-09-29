#!/usr/bin/env python3
"""Portable standalone FE replay runner (Python 3.9+, standard library only).

Organization: EPFL INL
Author: Yuyang Chen
Last modified: 2026-09-29
No repository outside this script's directory is imported or accessed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'sim_data'
BUILD = ROOT / 'sim_build'
PASS = re.compile(r'TB_PASS frames=(\d+) tokens=(\d+) bytes=(\d+) cycles=(\d+) max_frame_cycles=(\d+)')


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_data(algorithms):
    """Check identities and formats BEFORE building; missing fixtures are fatal."""
    manifest = json.loads((ROOT / 'tb/data_manifest.json').read_text())
    names = {'stream_frames.memh'}
    for name in algorithms:
        names.update((name + '_payload.memh', name + '_stream_golden.memh'))
    errors = []
    checked = {}
    for item in manifest['files']:
        if item['name'] not in names:
            continue
        path = DATA / item['name']
        if not path.is_file():
            errors.append(f"Missing sim_data/{item['name']} (original project: {item['source']})")
            continue
        words = path.read_text(encoding='ascii').split()
        if len(words) != item['words'] or any(
                not re.fullmatch('[0-9a-fA-F]{' + str(item['hex_digits']) + '}', word)
                for word in words):
            errors.append(f"Wrong word count/hex width: {item['name']}")
            continue
        if sha256(path) != item['sha256']:
            errors.append(f"SHA-256 mismatch: {item['name']}; use the matching supplied fixture")
            continue
        if item['hex_digits'] == 2 and any(int(word, 16) > 63 for word in words):
            errors.append(f"Golden value outside 0..63: {item['name']}")
            continue
        checked[item['name']] = item['sha256']
        print(f"DATA_OK {item['name']}: {len(words)} words", flush=True)
    if errors:
        raise RuntimeError('\n'.join(errors) + '\nSee sim_data/README_CN.md; no data is downloaded automatically.')
    return checked


def run_logged(command, log_path, timeout, stream=False, prefix=''):
    """Log exact commands. Stream simulation progress and enforce wall timeout."""
    with log_path.open('w', encoding='utf-8') as log:
        log.write('COMMAND ' + json.dumps(command, ensure_ascii=False) + '\n')
        log.flush()
        if not stream:
            result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                    timeout=timeout)
            if result.returncode:
                tail = '\n'.join(log_path.read_text(encoding='utf-8', errors='replace').splitlines()[-35:])
                raise RuntimeError(f'Build failed: {log_path}\n{tail}')
            return
        with subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                              errors='replace', bufsize=1) as process:
            expired = threading.Event()

            def stop_on_timeout():
                expired.set()
                process.kill()

            timer = threading.Timer(timeout, stop_on_timeout)
            timer.daemon = True
            timer.start()
            try:
                for line in process.stdout:
                    log.write(line)
                    log.flush()
                    print(prefix + line.rstrip(), flush=True)
                code = process.wait()
            finally:
                timer.cancel()
                if process.poll() is None:
                    process.kill()
                    process.wait()
            if expired.is_set():
                raise RuntimeError(f'Simulation wall timeout ({timeout}s): {log_path}')
            if code:
                raise RuntimeError(f'Simulator exit code {code}: {log_path}')


def run_case(name, layout, args, simulator):
    tag = f'{simulator}_{name}_{layout}_n{args.frames}_gap{args.frame_interval}'
    directory = BUILD / tag
    directory.mkdir(parents=True, exist_ok=True)
    values = {'OSSM': int(name == 'ossm'), 'FREQUENCY_MAJOR': int(layout == 'frequency'),
              'FRAME_COUNT': args.frames, 'FRAME_INTERVAL': args.frame_interval}
    if simulator == 'iverilog':
        executable = directory / 'simulation.vvp'
        compile_command = ['iverilog', '-g2005', '-s', 'tb_fe_stream', '-o', str(executable)]
        compile_command += [f'-Ptb_fe_stream.{key}={value}' for key, value in values.items()]
        compile_command += ['-f', 'all.f', 'tb/tb_fe_stream.v']
        run_command = ['vvp', str(executable)]
    else:
        obj = directory / 'obj'
        executable = obj / ('sim_fe.exe' if sys.platform == 'win32' else 'sim_fe')
        compile_command = ['verilator', '--binary', '--timing', '--language', '1364-2005',
                           '--top-module', 'tb_fe_stream', '--Mdir', str(obj), '-o', executable.name,
                           '-j', '2']
        if args.vcd:
            compile_command += ['--trace', '--trace-depth', '1']
        compile_command += [f'-G{key}={value}' for key, value in values.items()]
        compile_command += ['-f', 'all.f', 'tb/tb_fe_stream.v']
        run_command = [str(executable)]
    for key, filename in [('OUTPUT_FILE', 'output.csv'), ('INPUT_TRACE', 'inputs.csv'),
                          ('PARAM_TRACE', 'parameters.csv')]:
        run_command.append(f'+{key}={(directory / filename).as_posix()}')
    if args.vcd:
        run_command.append(f'+VCD={(directory / "wave.vcd").as_posix()}')
    print(f'BUILD {tag}', flush=True)
    started = time.monotonic()
    run_logged(compile_command, directory / 'build.log', args.timeout)
    print(f'RUN {tag}', flush=True)
    run_logged(run_command, directory / 'simulation.log', args.timeout, True, f'[{name}/{layout}] ')
    log = (directory / 'simulation.log').read_text(encoding='utf-8', errors='replace')
    matches = PASS.findall(log)
    if len(matches) != 1 or 'TB_FAIL' in log or 'MISMATCH' in log or 'ERROR:' in log:
        raise RuntimeError(f'No unique successful completion: {directory / "simulation.log"}')
    frames, tokens, byte_count, cycles, max_frame = map(int, matches[0])
    if (frames, tokens, byte_count) != (args.frames, args.frames // 50, args.frames // 50 * 320):
        raise RuntimeError(f'Wrong final counts: {matches[0]}')
    result = dict(passed=True, algorithm=name, layout=layout, simulator=simulator,
                  frames=frames, tokens=tokens, bytes=byte_count, cycles=cycles,
                  max_frame_cycles=max_frame, frame_interval=args.frame_interval,
                  mismatched_bytes=0, seconds=round(time.monotonic() - started, 3),
                  directory=str(directory.relative_to(ROOT)))
    (directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--algorithm', choices=['both', 'cwt', 'ossm'], default='both')
    parser.add_argument('--layout', choices=['both', 'channel', 'frequency'], default='both')
    parser.add_argument('--simulator', choices=['auto', 'verilator', 'iverilog'], default='auto')
    parser.add_argument('--frames', type=int, default=1000, help='Multiple of 50, within 50..1000')
    parser.add_argument('--frame-interval', type=int, default=0,
                        help='Minimum cycles between frames; 0=ready-paced, 200000=2 ms')
    parser.add_argument('--timeout', type=int, default=7200, help='Wall timeout per build/run in seconds')
    parser.add_argument('--vcd', action='store_true', help='Dump public TB signals to wave.vcd')
    parser.add_argument('--check-data-only', action='store_true')
    args = parser.parse_args()
    if args.frames < 50 or args.frames > 1000 or args.frames % 50:
        parser.error('--frames must be a multiple of 50 within 50..1000')
    if args.frame_interval < 0 or args.frame_interval > 1000000 or args.timeout <= 0:
        parser.error('frame interval must be 0..1000000 and timeout must be positive')
    algorithms = ['cwt', 'ossm'] if args.algorithm == 'both' else [args.algorithm]
    layouts = ['channel', 'frequency'] if args.layout == 'both' else [args.layout]
    BUILD.mkdir(exist_ok=True)
    summary_path = BUILD / 'summary.json'
    summary = dict(passed=False, status='running', runs=[],
                   golden_kind='Python fixed-point continuous-stream reference (not float golden)',
                   functional_scope='N=50 real stream; both layouts; parameter gaps; periodic output stalls')
    if not args.check_data_only:
        # Never leave an earlier PASS summary looking like the current run passed.
        summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    try:
        summary['data_sha256'] = check_data(algorithms)
        if args.check_data_only:
            print('PASS: all required fixtures have matching format and SHA-256')
            return 0
        simulator = args.simulator
        if simulator == 'auto':
            simulator = 'verilator' if shutil.which('verilator') else 'iverilog'
        required = ['verilator'] if simulator == 'verilator' else ['iverilog', 'vvp']
        missing = [name for name in required if not shutil.which(name)]
        if missing:
            raise RuntimeError('Missing tools on PATH: ' + ', '.join(missing) + '. See README_CN.md.')
        version_command = ['verilator', '--version'] if simulator == 'verilator' else ['iverilog', '-V']
        version = subprocess.run(version_command, capture_output=True, text=True, timeout=15)
        summary['simulator_version'] = (version.stdout or version.stderr).splitlines()[0]
        paths = [ROOT / line for line in (ROOT / 'all.f').read_text().splitlines() if line.strip()]
        paths += [ROOT / 'tb/tb_fe_stream.v', ROOT / 'run_testbench.py', ROOT / 'tb/data_manifest.json']
        summary['source_sha256'] = {str(p.relative_to(ROOT)): sha256(p) for p in paths}
        for name in algorithms:
            for layout in layouts:
                summary['runs'].append(run_case(name, layout, args, simulator))
                summary_path.write_text(json.dumps(summary, indent=2) + '\n')
        if any(sha256(ROOT / p) != value for p, value in summary['source_sha256'].items()):
            raise RuntimeError('Source changed during simulation; rerun for a consistent report')
        summary.update(passed=True, status='complete',
                       compared_bytes=sum(run['bytes'] for run in summary['runs']))
        summary_path.write_text(json.dumps(summary, indent=2) + '\n')
        print(f"PASS: {len(summary['runs'])} runs, {summary['compared_bytes']} identical bytes. "
              f'Report: {summary_path}', flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        summary.update(status='failed', error=str(exc))
        if not args.check_data_only:
            summary_path.write_text(json.dumps(summary, indent=2) + '\n')
        print('FAIL: ' + str(exc), file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
