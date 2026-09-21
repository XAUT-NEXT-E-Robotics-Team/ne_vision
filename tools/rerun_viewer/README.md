# Rerun 双视图启动

安装 Rerun Viewer 0.37.1 后运行（可从任意工作目录执行）：

```bash
bash tools/rerun_viewer/bringup.sh
```

默认监听 9876 端口，加载 `nv_dual.rbl`，对应 `rerun_debug.name: nv`：

- 左侧 3D 场景：原点 `/rerun_debug/imu`。
- 右侧图像与 3D box 投影：原点 `/rerun_debug/imu/camera/frame`，包含图像、2D 标注和 `/rerun_debug/imu/detected_armor3d`。

程序需发布 `imu/camera` 的 TF，以及 `imu/camera/frame` 的 Pinhole 和图像。
启动脚本只配置 Viewer，不启动视觉程序；启动后再连接视觉程序。

原有 `nv.rbl` 保留。需要使用它时：

```bash
RBL_FILE_NAME=nv.rbl bash tools/rerun_viewer/bringup.sh
```

`RERUN_BIN`、`RERUN_PORT`、`RERUN_MEMORY_LIMIT`、`RERUN_SERVER_MEMORY_LIMIT` 可通过环境变量覆盖。
脚本后的参数会继续传给 Viewer。

修改应用 ID 或实体根路径时，使用匹配版本的 Python SDK 重新生成蓝图：

```bash
python3 -m pip install rerun-sdk==0.37.1
python3 tools/rerun_viewer/generate_blueprint.py --app-id nv --root /rerun_debug
```

日常启动使用已生成的蓝图，不依赖 Python SDK。
