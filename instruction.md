# 陀螺回放

我这边拿到的是二号转台的一段标定记录和一段回放记录。标定段有转台真实角速度、陀螺电压和温度。回放段没有真实角速度，只能靠标定结果去扣零偏、温漂和轴间增益。标定表里偶尔会有电压尖峰。还有些样品的增益很小，矩阵行列式贴近零，但矩阵没有坏。厂内标称矩阵也一并给了，标定样本不够时可以用。

输入字段和输出字段按下面这份交接单写：

```text
run_id：字符串，原样带回
calibration.nominal_matrix：厂内标称增益矩阵
calibration.samples：标定数组，每条含 raw_gyro、true_rate、temperature_c
`raw_gyro` 是传感器原始输出，单位 V
initial_quaternion：[w, x, y, z]，姿态表示从机体系到世界系的旋转
samples：回放数组，每条含 t、raw_gyro、temperature_c，t 单位为秒
每个采样值保持到下一个采样时刻，最后一个采样点不参与积分
checkpoints：输出时刻，均与某个采样时刻相同

标定模型：
raw_gyro = matrix * true_rate + bias_at_25c + bias_temp_coefficient * (temperature_c - 25)
`matrix` 把真实机体系角速度映射为陀螺原始读数
校准样本可能出现孤立尖峰
最大单轴残差超过 0.15 rad/s 时，拟合校准模型时不计入异常标定样本

输出：程序向标准输出写一个 JSON 对象，字段为 `run_id`、`calibration` 和 `attitudes`，不要输出其它文字
calibration：matrix（3×3）、bias_at_25c（三轴）、bias_temp_coefficient（三轴）、rejected_calibration_indices
`bias_at_25c` 是 25°C 的零偏
`bias_temp_coefficient` 是零偏随摄氏温度的变化率
`rejected_calibration_indices` 按 0 开始的原始顺序列出异常标定样本
回放样本带有各自的温度读数
标定矩阵可能接近奇异，但行列式不为零时仍应按可逆处理
标称矩阵只用于激励不足时回退
attitudes：按 checkpoints 顺序输出，每项含 t 和 q_wxyz
四元数取单位长度，按 [w, x, y, z] 排列，w 取非负的等价表示
参数个数不对时退出码为 2，不输出 JSON
```

容器里只跑 Python 标准库，不走网络。同一份输入重复跑，结果要一致。
