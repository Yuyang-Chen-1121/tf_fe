# 本地仿真数据

| 文件名 | 原工程中的来源 | 内容 |
| --- | --- | --- |
| `cwt_payload.memh` | `hdl/coeff/cwt_payload.memh` | 1117 个 24-bit CWT 配置字 |
| `ossm_payload.memh` | `hdl/coeff/ossm_payload.memh` | 6785 个 24-bit OSSM 配置字 |
| `stream_frames.memh` | `hdl/vectors/stream_frames.memh` | 1000 帧真实输入，每帧 64 个 signed12 样本 |
| `cwt_stream_golden.memh` | `hdl/vectors/cwt_stream_golden.memh` | CWT 的 20 个整数参考 token，共 6400 byte |
| `ossm_stream_golden.memh` | `hdl/vectors/ossm_stream_golden.memh` | OSSM 的 20 个整数参考 token，共 6400 byte |

```sh
python run_testbench.py --check-data-only
```

文件全部是纯十六进制文本，每行一个字，无 `0x` 前缀：

- 配置文件：每行 6 个十六进制字符，按参数 RAM 相对地址从 0 开始排列。24-bit 字内的有符号字段已经按补码打包。
- 输入文件：每行 192 个十六进制字符，即 768 bit。最右侧 3 个字符是 channel 0 的 signed12 码，其左边依次是 channel 1、channel 2，最左侧是 channel 63。
- 参考文件：每行 2 个十六进制字符，值为 `00`～`3f`。按 token、channel、frequency 排列；每个 token 连续 320 行。

输入来自原工程 `test_ecog` 的第 0 个窗口及第 1 个窗口的前 250 帧，是有意拼接的 1000 帧连续状态测试流。文件已完成 FE 输入编码，直接送端口即可，不需要再降采样或在线归一化；归一化由 RTL 执行。

参考输出来自原工程 Python **定点整数**连续流模型，平均长度为 50，从第一个输入样本开始累计。它用于要求逐字节完全一致，不是浮点 FE 直接量化后的误差参考。原始生成入口是 `hdl/tools/export_stream_vectors.py`，生成过程需要原工程，但本 testbench 的重放过程不需要。

