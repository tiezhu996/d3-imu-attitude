# 惯性测量单元标定与短时姿态回放

台架组用三轴转台录制陀螺标定数据，再让同一只传感器回放一段短时运动。不同器件之间的轴间增益、零偏和温漂差别很大，标定文件里偶尔还会出现记录尖峰。转台的几组激励可能并不独立，真实角速度之间会存在很强的相关性。现有工具按理想三轴直接积分，回放姿态和光学基准对不上。

## 现状

`/app/main.py` 能读试验文件并输出姿态。标定样品的真实转台角速度、原始陀螺读数和温度都在输入里；回放样品只有原始读数和温度。回放样本带有各自的温度读数。当前工具没有从标定数据中恢复轴间增益、参考温度零偏和温漂，也没有排除尖峰。

## 要交付的内容

修好 `/app/main.py`。用法是 `python3 /app/main.py <输入文件>`，输入文件是一个 JSON 对象：

- `run_id`：字符串。
- `calibration.samples`：标定数组。每项含 `raw_gyro`、`true_rate`、`temperature_c`。`raw_gyro` 是传感器原始输出，单位为 V。
- `initial_quaternion`：`[w, x, y, z]`，姿态表示从机体系到世界系的旋转。
- `samples`：按时间递增的回放数组。每项含 `t`（秒）、`raw_gyro`、`temperature_c`。`raw_gyro` 是传感器原始输出。每个采样值保持到下一个采样时刻；最后一个采样点不参与积分。
- `checkpoints`：需要输出的时刻，均与某个采样时刻相同。

标定模型为：

```text
raw_gyro = matrix * true_rate + bias_at_25c + bias_temp_coefficient * (temperature_c - 25)
```

`matrix` 把真实机体系角速度映射为陀螺原始读数。校准样本可能出现孤立尖峰；如果某条样本在拟合模型下的最大单轴残差超过 `0.15 rad/s`，就视为异常，不参与最终拟合。

程序向标准输出写一个 JSON 对象，字段为 `run_id`、`calibration` 和 `attitudes`，不要输出其它文字。`calibration` 含：

- `matrix`：3×3 数组。
- `bias_at_25c`：25°C 的零偏，三个数。`bias_at_25c` 是 25°C 的零偏。
- `bias_temp_coefficient`：零偏随摄氏温度的变化率，三个数。`bias_temp_coefficient` 是零偏随摄氏温度的变化率。
- `rejected_calibration_indices`：按 0 开始的原始顺序列出异常标定样本。`rejected_calibration_indices` 按 0 开始的原始顺序列出异常标定样本。拟合校准模型时不计入异常标定样本。

`attitudes` 按 `checkpoints` 的顺序排列，每项含 `t` 和 `q_wxyz`。姿态使用单位四元数，按 `[w, x, y, z]` 排列，取 `w >= 0` 的等价表示。参数个数不对时退出码为 2，不输出 JSON。

## 约束

只使用容器内已有的 Python 标准库，不访问网络。输入最多 200 条标定样品和 12000 条回放采样，单次运行要在 20 秒内结束。同一份输入重复运行，输出必须一致。
