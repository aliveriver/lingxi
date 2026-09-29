# 相机

## 数据源

| 名称 | Topic | 格式/官方频率 |
| --- | --- | --- |
| `rgbd_front_rgb` | `/aima/hal/sensor/rgbd_head_front/rgb_image/compressed` | MJPEG，1280x720，30 Hz |
| `rgbd_front_depth` | `/aima/hal/sensor/rgbd_head_front/depth_image` | 16UC1，mm，1280x720，30 Hz |
| `stereo_front_left/right` | `/aima/hal/sensor/stereo_head_front_{left,right}/rgb_image/compressed` | JPEG，X2 Ultra 2064x1552，10 Hz |
| `head_rear` | `/aima/hal/sensor/rgb_head_rear/rgb_image/compressed` | JPEG，X2 Ultra 2064x1552，10 Hz |
| `head_front_center` | `/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed` | 实机发现；规格待测 |

v1.1.4 升级后短时复验：RGB-D 压缩 RGB 和 depth 均约 30 Hz；depth 为 1280x720、`16UC1`，压缩 RGB 消息格式为 JPEG。频率测量只覆盖约 7 秒，不代表长时间稳定性。

实际尺寸从消息/图像读取，不硬编码。不同硬件版本可能是 2048x1536。原始 RGB 单路约 80-90 MB/s，只应在 PC2 本机订阅；Web 默认使用压缩流。

```bash
uv run x2 --config config/x2.yaml capture rgbd_front_rgb snapshots/front.jpg
```

```python
with X2Client("config/x2.yaml") as x2:
    frame = x2.camera_frame("rgbd_front_rgb")
    print(frame.timestamp, frame.width, frame.height, frame.encoding)
```

在 `config/x2.yaml` 的 `ros.camera_streams` 中列出要订阅的流。每增加订阅都会消耗 DDS 与解码资源，机器人站立/走跑时不要短时间批量启动节点。

RGB 与深度独立发布且不是同步帧对。深度单位为毫米；D2C 硬件对齐关闭，需使用内参和 `/tf_static` 在应用层配准。
