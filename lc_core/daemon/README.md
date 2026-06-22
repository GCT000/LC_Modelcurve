# lc_daemon — LC-CurveModel 采集解算编排器

监听 `data_collector` 产出的数据目录,自动把每帧的图像 / 点云写入
`model1.yaml`,再调用 `curve` 逐帧解算。面向长期无人值守运行,内置滑动窗口、
重启去重、解算超时、日志清理四项加固。

## 数据流

```
data_collector ──> base/data/<时间戳>/        lc_daemon (本程序)        curve
   每帧产出:          extracted.pcd   ──监听──>  改写 model1.yaml  ──调用──>  解算
                      image.png                  逐帧顺序触发              输出到帧目录
```

- 时间戳文件夹名格式:`yy-mm-dd-HH-MM-SS`(2 位年,6 段),字典序即时间序。
- 每帧采集产物:`extracted.pcd`(点云)+ `image.png`(图像)。
- 预处理 3 件套(仅首帧文件夹需要,文件名固定):
  | 文件 | 写入 yaml 字段 | 说明 |
  |------|----------------|------|
  | `1.pcd` | `lidar_points_path` | 预处理点云 |
  | `image_points_1.txt` | `selected_points` | 人工选点 |
  | `endpoint.txt` | `end_point` | 端点坐标,格式 `x,y,z`(逗号分隔) |

## 解算逻辑

**首帧**:倒序扫描数据目录,取第一个含预处理 3 件套的文件夹。比它更早的文件夹
永久忽略。首帧解算时清空 `last_image_path`,使 curve 走 first_time 分支
(靠 `selected_points` 初始化曲线)。

**后续帧**:首帧之后、按时间升序逐个解算。`last_image_path` 指向前一帧图像,
使 curve 走光流跟踪分支。

## 首帧自动计算的 yaml 字段

首帧解算前,编排器会读取 `1.pcd` 的全部点云,计算包围盒并写入两个字段
(只在首帧计算,后续帧沿用):

- `rectang_size: [x_min, x_max, y_min, y_max, z_min, z_max]` —— 点云三轴的
  最小/最大值,顺序固定。
- `xy_interval`:取 `{y_min, y_max, endpoint.txt 的中间值(y)}` 三者,
  `start` = 最小值,`end` = 最大值。

## 编译

已注册于 `lc_core/CMakeLists.txt`,产物为 `bin/lc_daemon`(链接 PCL)。

```bash
cd /home/gct/LC_Modelcurve/build
cmake .                                   # 改过链接库后需重新配置一次
cmake --build . --target lc_daemon
```

## 运行

> **重要(LD_LIBRARY_PATH)**:`lc_daemon` 和 `curve` 都依赖 PCL,但都**不需要**
> 海康 MVS SDK。若环境变量 `LD_LIBRARY_PATH` 含 `/opt/MVS/lib/64`(相机采集端
> hik_camera 需要),MVS 自带的 `libusb-1.0.so.0` 会顶掉系统完整版,导致
> `libpcl_io.so: undefined symbol: libusb_set_option` 报错。运行编排器时需用
> 不含 MVS 路径的库环境(见下)。

所有参数都有默认值,最简运行:

```bash
LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v /opt/MVS | paste -sd: -) \
  /home/gct/LC_Modelcurve/bin/lc_daemon -alsologtostderr
```

显式指定参数的完整示例:

```bash
/home/gct/LC_Modelcurve/bin/lc_daemon \
  -data_root=/home/gct/LC_Modelcurve/data \
  -yaml_path=/home/gct/LC_Modelcurve/config/whu/model1.yaml \
  -curve_bin=/home/gct/LC_Modelcurve/bin/curve \
  -daemon_log_dir=/home/gct/LC_Modelcurve/data/lc_daemon_logs \
  -state_file=/home/gct/LC_Modelcurve/data/.lc_state \
  -poll_sec=2 \
  -ready_timeout_sec=120 \
  -keep_frames=50 \
  -curve_timeout_sec=1200 \
  -log_clear_days=7 \
  -hash_size_limit_mb=1 \
  -alsologtostderr
```

程序常驻运行,`Ctrl+C` 退出。

## 参数说明

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `-data_root` | `/home/gct/LC_Modelcurve/data` | 监听的数据根目录(即 data_collector 的 `base/data`) |
| `-yaml_path` | `config/whu/model1.yaml` | 要改写并传给 curve 的配置文件 |
| `-curve_bin` | `bin/curve` | 解算器可执行文件路径 |
| `-daemon_log_dir` | `data/lc_daemon_logs` | 编排器自身日志目录 |
| `-state_file` | `data/.lc_state` | 已解算帧的状态持久化文件(重启去重用) |
| `-poll_sec` | `2` | 轮询数据目录的间隔(秒) |
| `-ready_timeout_sec` | `120` | 文件夹出现后,等 `extracted.pcd`+`image.png` 写完的最长秒数 |
| `-keep_frames` | `50` | 滑动窗口:磁盘上最多保留的帧文件夹数 |
| `-curve_timeout_sec` | `1200` | 单次 curve 解算的最长运行时长(秒),超时则 kill |
| `-log_clear_days` | `7` | 每隔 N 天清空一次编排器日志目录 |
| `-hash_size_limit_mb` | `1` | 签名时:小于等于此大小的文件用内容哈希,否则用 mtime+size |
| `-alsologtostderr` | — | 日志同时输出到终端,便于观察 |

## 长期运行加固

**1. 磁盘滑动窗口**
每次解算后,帧文件夹数超过 `-keep_frames` 即删除最老的。**首帧目录永久保护**,
因为后续帧的 yaml 仍引用首帧的预处理文件。

**2. 重启去重 / 重新预处理检测**
已解算的帧记录签名到 `-state_file`。重启后:
- 文件未变 → 跳过,不重复解算。
- 首帧的预处理文件发生变化(重新预处理)→ 清空状态,从首帧整链重新解算
  (因为后续帧的光流都是从首帧链式传递,首帧变则整链失效)。

签名策略(混合):小文件(`endpoint.txt`、`image_points_1.txt`)用内容哈希,
精确;大文件(`1.pcd`)用 mtime+size,免读全文件。

**3. curve 解算超时**
单帧 curve 运行超过 `-curve_timeout_sec` 即被 kill(先 SIGTERM 后 SIGKILL),
记录失败并继续下一帧,不阻塞流水线。
注意:此超时只计 curve 进程本身的运行时长,与「两帧数据之间隔多久」无关——
数据采集间隔再长,编排器也只是空转等待,不会被 kill。

**4. 日志清理**
主循环每满 `-log_clear_days` 天清空一次 `-daemon_log_dir`。

## 解算结果保存位置

编排器把 `res_path` 等输出字段改写到当前帧目录,因此每帧的解算结果保存在
`base/data/<时间戳>/` 下,主要包括:

- `result.txt` —— 最终测距结果
- `final_line_points.pcd` / `.txt` —— 最终曲线点云
- `original/middle/final_output_lidar_points.txt`、对应 `.pcd`
- `matched_points.pcd` / `filtered_cloud.pcd` / `matched_points.txt`
- `projection_1.jpg`、`lidar_points_Q.txt`、`visual_points_R.txt`
- `log/` —— 该帧 curve 的 glog 日志

注:curve 源码里 `temp_path`(`data/temp/`)下的中间可视化图是全局覆盖写,
不分帧保存,也不随滑动窗口增长。

## 相关文件

| 文件 | 说明 |
|------|------|
| `lc_daemon.cpp` | 编排器主程序 |
| `yaml_patcher.h` | yaml 字段改写、endpoint.txt 解析、rectang_size/xy_interval 写入 |
| `pcd_bbox.h` | 读取 1.pcd 计算点云包围盒(PCL) |
| `frame_signature.h` | 混合签名(内容哈希 / mtime+size) |
| `*.bak.*` | 改造前的原始备份 |
