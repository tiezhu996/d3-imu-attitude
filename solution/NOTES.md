# 参考解说明

## 三个难点

| 编号 | 错误认知 | 正确知识 | 对应测试（tests/scoring.json） |
|---|---|---|---|
| difficulty_1 | 认为三轴增益是单位矩阵 | 用标定数据的 `true_rate` 回归完整 3×3 矩阵，回放时用逆矩阵校正原始角速度 | `test_matrix_recovers_cross_axis_gain`；`test_matrix_inverse_drives_attitude`；`test_matrix_handles_rotated_initial_frame` |
| difficulty_2 | 只扣除一个常数零偏 | 每轴同时拟合 25°C 截距和温度斜率，按回放样品自己的温度扣温漂 | `test_bias_at_reference_temperature`；`test_temperature_coefficient_is_recovered`；`test_temperature_correction_drives_attitude` |
| difficulty_3 | 让尖峰参与普通最小二乘 | 按最大单轴残差 0.15 rad/s 剔除异常样本，重复拟合到索引稳定，再用干净样本恢复零偏和矩阵 | `test_rejected_indices_are_exact`；`test_outlier_rejection_recovers_bias`；`test_outlier_rejection_drives_attitude` |

## 参考解文件

- `solution/app/main.py`：完整参考实现。
- `solution/solve.sh`：安装参考实现，并支持 `HTB_SKIP` 用精确片段退回对应错误模式。
- 环境原始文件仍是 `environment/app/main.py`，镜像只复制 `environment/`。

## 关键步骤

1. 读取标定样品和回放样品，保留原始 0-based 索引。
2. 对每个原始轴，用 `[true_x,true_y,true_z,1,T-25]` 做线性最小二乘。
3. 拟合异常样本前先形成初始模型，按最大单轴残差 0.15 rad/s 找尖峰，剔除后重新拟合。
4. 用 `M^{-1}(raw-b-cΔT)` 得到回放段的真实机体系角速度。
5. 每个采样区间用 `q = q ⊗ Exp(ωdt)` 传播，只输出题面要求的字段。

## 核心公式与独立复算

- 解析推导：`research/derivations.md` 第 1–3 节。
- 独立数值复核：`research/verify_calibration.py`，另有 `research/verify_attitude.py` 验证四元数传播。高相关激励用 Householder QR 拟合。
- 独立 OLS/QR 输出：`rejected=[11, 29, 47, 66]`，矩阵误差 `2.603e-06`，零偏误差 `6.449e-07`，温漂误差 `8.352e-08`，条件化 QR 误差 `1.158e-07`。
- 测试容差：矩阵 `5e-5`，零偏 `5e-5 rad/s`，温漂 `2e-5 rad/s/K`，姿态 `2e-4 rad`。

## 容差依据

- 正常噪声：标定样品使用 2e-5 rad/s 的确定性小扰动。
- 独立复算误差：远小于测试容差。
- 典型错误：单位矩阵忽略轴间耦合；常数零偏忽略温度；未剔除尖峰会改变零偏并污染回放姿态。
- 取值理由：容差高于正常拟合抖动，低于任一错误模式的偏差。

## 权重依据

- gate 测试（不计分）：输出字段契约、参数错误退出码。
- difficulty_1 合计 0.40：完整轴间矩阵和逆矩阵决定主回放结果。
- difficulty_2 合计 0.30：温漂是独立传感器标定量。
- difficulty_3 合计 0.30：异常样本是独立数据清洗和稳健拟合路径。

## 数据来源

- 真实数据：未使用生产数据；温度系数范围参考 Bosch BMI088 数据表。
- 生成数据：由 `tests/test_outputs.py` 的确定性三角函数工况生成，不含随机种子；异常尖峰位置和目标参数固定在测试文件中。
- 复现命令：`python3 -I research/verify_calibration.py`，`python3 -I research/verify_attitude.py`。

## 答案隔离

- [x] 镜像内没有 `tests/`、`solution/`、`.git/`、缓存和备份。
- [x] 注释、文件名、变量名、数据元信息没有答案提示。
- [x] Dockerfile 没有先复制正确版本再改坏。
