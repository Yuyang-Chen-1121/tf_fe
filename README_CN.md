# CWT / OSSM 汇报版 RTL

单位：EPFL INL  
作者：Yuyang Chen  
最后修改：2026-09-29

## 1. 文件组织

```text
rtl_presentation/
├── CWT/
│   ├── cwt_feature_engine.v   独立 CWT 顶层、控制器、地址与存储
│   ├── fe_cwt_mac.v          INT12 复数核点积
│   └── fe_sqrt.v             幅值开方
├── OSSM/
│   ├── ossm_feature_engine.v 独立 OSSM 顶层、控制器、地址与存储
│   └── fe_ossm_step.v        基线、预测、共同残差、修正和功率
├── common/
│   ├── fe_config_store.v    参数双 bank 加载与激活
│   ├── fe_stream_control.v  平均区间与运行状态计数
│   ├── fe_input_affine.v    signed12 输入标准化
│   ├── fe_output_quant.v    平均/量化的整数除法
│   ├── fe_token_output.v    双 token bank、布局与输出握手
│   ├── fe_round.v           RNE 舍入和饱和
│   └── fe_storage.v         标准 DFF 与同步 RAM
├── cwt.f                    仅 CWT 所需的源文件清单
├── ossm.f                   仅 OSSM 所需的源文件清单
└── all.f                    两种算法的联合源文件清单，common 只列一次
```

`common` 表示共用模块定义；两个引擎分别实例化自己的公共模块。同时实例化两个引擎时，不会自动共用物理 RAM、除法器或输出队列。

## 2. 状态机的展示方式

两个外层引擎，以及 OSSM 内部的 `fe_ossm_step`，都采用传统三段式：

1. `always @(posedge clk or negedge rst_n)` 保存当前状态，处理 reset/clear。
2. 第二个 `always @*` 以 `case` 列出下一状态；默认保持，非法编码回到 IDLE。
3. 第三个 `always @*` 以 `case` 产生当前状态的读地址、操作使能、握手、错误采样和循环控制。


### CWT 主线

```text
IDLE → NORM_INV / NORM_BIAS（遍历通道）
     → KERNEL_LOAD / TAP_ACCUM（遍历 tap）
     → MAC_RESULT → SQRT_WAIT（遍历频率和通道）
     → PROCESS_START → POOL（遍历 320 个特征）
     → 若为平均末帧：DEN_LOW → DEN_HIGH → DIV_START → DIV_WAIT
     → 完整 token 才 COPY → FINISH → IDLE
```

### OSSM 主线

```text
IDLE → NORM_INV / NORM_BIAS（遍历通道）
     → ROTATE_LOAD → PROCESS_START → GAIN_LOAD
     → STEP_START → STEP_WAIT
     → POOL（当前通道的五个功率；末帧还要逐特征量化）
     → 下一个通道，或完整 token COPY → FINISH → IDLE
```
`fe_ossm_step` 中另一个三段式状态机，控制共享乘法器的 `LOAD → MULTIPLY → STORE`，保留每通道 178 个内部运算周期。

## 3. 综合与文件选择

在 Efinity 中仅导入本目录中对应 `.f` 列出的 Verilog 文件，并选择：

- CWT 顶层：`cwt_feature_engine`。
- OSSM 顶层：`ossm_feature_engine`。