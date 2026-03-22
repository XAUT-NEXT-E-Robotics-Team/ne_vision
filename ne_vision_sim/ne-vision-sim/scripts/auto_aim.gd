
extends Node

@onready var auto_aim_viewport = $AutoAimViewport
@onready var gimbal = $AutoAimViewport/Gimbal
@export var auto_aim_result_frame : TextureRect
# 假设你想在 Inspector 中动态调整这个额外的 Yaw 角度（单位：度）
@export var extra_yaw_degrees: float = 0.0
@onready var nv_gd = NeVisionGd.new()
var fps_label: Label

# 在类变量中缓存 texture
var _display_texture: ImageTexture

# Called when the node enters the scene tree for the first time.
func _ready() -> void:

    var config_path = ProjectSettings.globalize_path("res://gdextensions/ne_vision_gd/config/config_sim.yaml")
    nv_gd.start(config_path)

    # 创建一个用于显示 FPS 的 Label
    fps_label = Label.new()
    fps_label.name = "FPSLabel"
    fps_label.add_theme_color_override("font_color", Color.GREEN)
    fps_label.add_theme_font_size_override("font_size", 24)
    fps_label.position = Vector2(10, 10) # 左上角位置

    # 如果 auto_aim_result_frame 已经就绪，作为其子节点添加，这样它会跟随 TextureRect
    if auto_aim_result_frame:
        auto_aim_result_frame.add_child(fps_label)
    else:
        add_child(fps_label)


func _process(delta: float) -> void:
    # 1. 获取图像 (这一步依然慢，是硬伤)
    var img = auto_aim_viewport.get_texture().get_image()

    # 2. 处理图像 (假设这里进行了某种可视化处理)
    nv_gd.updata_frame(img)
    nv_gd.get_visualize_frame(img)

    # 更新 FPS 显示
    if fps_label:
        fps_label.text = "FPS: %d" % Engine.get_frames_per_second()

func _physics_process(delta: float) -> void:
# 1. 获取 Godot 坐标系下的全局姿态四元数
    var q_godot: Quaternion = gimbal.global_transform.basis.get_rotation_quaternion()
    
    # 2. 坐标系转换：转换为 Z 轴向上、X 轴向右的标准右手系
    var q_target = Quaternion(
        q_godot.x,   # 目标的 X 对应 Godot 的 X
        -q_godot.z,  # 目标的 Y 对应 Godot 的 -Z
        q_godot.y,   # 目标的 Z 对应 Godot 的 Y
        q_godot.w    # W 保持不变
    )
    
    # 3. 加速度和角速度保持为 0
    var acc_target: Vector3 = Vector3.ZERO
    var gyro_target: Vector3 = Vector3.ZERO

    # 4. 传入 C++ 扩展 (NeVisionGd)
    nv_gd.update_imu(acc_target, gyro_target, q_target, delta)
