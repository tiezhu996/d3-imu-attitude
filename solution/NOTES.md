# 参考解说明

## 三个难点

| 编号 | 错误认知 | 正确知识 | 对应测试（tests/scoring.json） |
|---|---|---|---|
| difficulty_1 | 把机体系角速度增量左乘到累计姿态上 | 姿态表示机体系到世界系时，恒角速度增量应在右侧组合：`q <- q ⊗ Exp(ω dt)` | `test_attitude_uses_body_rotation_after_initial_heading`；`test_cross_axis_body_rate_preserves_rotation_order`；`test_rotated_initial_frame_keeps_body_axis_motion` |
| difficulty_2 | 直接把第一个采样点当成整段零偏 | 首段和末段静止窗口分别求均值；首段结束后到末段开始前线性插值，首段之前不扣零偏 | `test_start_bias_uses_first_stationary_window`；`test_end_bias_uses_last_stationary_window`；`test_attitude_removes_linearly_drifting_bias` |
| difficulty_3 | 用一阶欧拉加归一化近似大角步长 | 每个采样值保持到下一时刻时，使用四元数指数映射 `Exp(ω dt)` | `test_exact_rotation_across_one_coarse_interval`；`test_fast_moderate_rate_rotation_uses_exact_step`；`test_full_turn_high_rate_keeps_attitude` |

## 参考解文件

- `solution/app/main.py`：完整参考实现。
- `solution/solve.sh`：安装参考实现，并支持 `HTB_SKIP` 用精确片段退回种子错误。
- 环境原始文件仍是 `environment/app/main.py`，镜像只复制 `environment/`。

## 关键步骤

1. 解析 JSON，按题面顺序读取采样、静止窗口和检查点。
2. 用首末静止窗口的陀螺均值得到 `bias_start` 和 `bias_end`。
3. 首段静止窗口之前不扣零偏；首段结束后到末段开始前，用窗口边界做线性插值。
4. 每个采样区间用 `q = q ⊗ Exp((gyro - bias) dt)` 传播，输出时归一化并取 `w >= 0`。
5. 只输出题面要求的四个字段。

## 核心公式与独立复算

- 解析推导：`research/derivations.md` 第 1–3 节。
- 独立数值复核：`research/verify_attitude.py`，使用 Rodrigues 旋转矩阵和 20000 步 RK4。
- 两条路径的差异：正确路径三组旋转误差显示为 `0.000e+00`；错误左乘误差 0.588255–1.385343 rad。
- 最终容差：D1 `2e-5 rad`，D2 零偏 `1e-12 rad/s`、姿态 `2e-5 rad`，D3 `2e-4 rad`。

## 容差依据

- 正常抖动上限：双精度解析/数值路径差异远小于 `1e-12 rad`，测试取宽松到 `2e-5`–`2e-4 rad`。
- 典型错误解偏差：D1 左乘 0.588–1.385 rad；D2 首个采样点当零偏造成 0.034862 rad；D3 欧拉 0.195269–1.671849 rad。
- 取值理由：容差高于浮点实现抖动，低于任一难点的典型错误一个数量级以上。

## 权重依据

- gate 测试（不计分）：输出字段契约、参数错误退出码。
- difficulty_1 合计 0.40：决定姿态主路径，三轴组合错误影响最大。
- difficulty_2 合计 0.30：影响传感器标定和运动段姿态，但不改变基础传播公式。
- difficulty_3 合计 0.30：只在粗采样高速数据上暴露，和 D1/D2 独立。

## 数据来源

- 真实数据：未使用生产数据。
- 生成数据：测试输入由 `tests/test_outputs.py` 的确定性构造器生成，不含随机种子；首末零偏和运动速率是题目定义的合成工况。
- 复现命令：`python3 -I research/verify_attitude.py`。

## 答案隔离

- [x] 镜像内没有 `tests/`、`solution/`、`.git/`、缓存和备份。
- [x] 注释、文件名、变量名、数据元信息没有答案提示。
- [x] Dockerfile 没有先复制正确版本再改坏。
