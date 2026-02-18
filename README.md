# 西安理工大学NEXT-E战队新自喵~

注意：自瞄核心部分与ROS2无关，原码可以通过修改cmake设置编译为GDExtension形式，ros2形式

## 用

以下内容在工程根目录下执行：

```
python3 nv.py build # 编译
source install/setup.sh # 设置环境变量
python3 nv.py run -h # 查看可以运行什么
python3 nv.py run ut # 运行所有单测
python3 nv.py run mt_auto_aim_video_test # 运行视频测试
# 仿真测试先不写了
``` 
视频测试中，按p可以暂停，按esc退出，按除以上外的任何键转到下一帧。（注意：该操作会影响KF）
