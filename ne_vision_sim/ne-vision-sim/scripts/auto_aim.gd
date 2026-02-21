
extends Node

@onready var auto_aim_viewport = $AutoAimViewport
@onready var gimbal = $AutoAimViewport/Gimbal
@export var auto_aim_result_frame : TextureRect

@onready var nv_gd = NeVisionGd.new()
var fps_label: Label

# 在类变量中缓存 texture
var _display_texture: ImageTexture

var last_pos: Vector3
var last_vel: Vector3
var last_basis: Basis

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
    last_pos = gimbal.global_position
    last_vel = Vector3.ZERO
    last_basis = gimbal.global_transform.basis

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
    var curr_pos = gimbal.global_position
    var curr_basis = gimbal.global_transform.basis

    # 线速度和加速度
    var curr_vel = (curr_pos - last_pos) / delta
    var acc_world = (curr_vel - last_vel) / delta
    # 将世界坐标系下的加速度转换到局部坐标系 (IMU 坐标系)
    # 假设 IMU 测量的是比力 (proper acceleration) (包含重力反作用力)
    # Godot 中的重力通常是 (0, -9.8, 0)
    var gravity = ProjectSettings.get_setting("physics/3d/default_gravity_vector") * ProjectSettings.get_setting("physics/3d/default_gravity")
    var acc_local = curr_basis.inverse() * (acc_world - gravity)

    # 角速度
    # 计算相对旋转
    # Angular velocity in world frame? No, typically gyro is local frame.
    # But let's verify math. R_cur = R_diff * R_prev.
    # Angular velocity vector w in body frame: w = (R_prev^T * (R_cur - R_prev) / dt) approx?
    # Or using quaternions: q_cur = q_diff * q_prev.
    # Simplified: angular velocity vector w ~ axis * angle / delta
    # The axis_angle calculated above is in global frame if we did curr * last.inv?
    # Let's use a simpler approach for body frame angular velocity.

    var q_curr = Quaternion(curr_basis)

    # q_diff represents rotation from last to curr in global frame if q_curr * q_last.inverse()
    # But we want angular velocity in local frame.
    # w_local = 2 * (q_last.inverse() * (q_curr - q_last) / delta) . xyz ?
    # Let's stick to basis for robustness against flipping quaternions

    var delta_rot = last_basis.inverse() * curr_basis
    var w_quat = delta_rot.get_rotation_quaternion()
    var angle = w_quat.get_angle()
    var axis = w_quat.get_axis()

    # 处理角度跨越或小角度
    if angle > PI:
        angle -= 2 * PI
    elif angle < -PI:
        angle += 2 * PI

    var gyro_local = axis * angle / delta

    # 坐标系变换:
    # Target X = Godot +X
    # Target Y = Godot -Z
    # Target Z = Godot +Y

    var acc_target = Vector3(acc_local.x, -acc_local.z, acc_local.y)
    var gyro_target = Vector3(gyro_local.x, -gyro_local.z, gyro_local.y)
    var q_target = Quaternion(q_curr.x, -q_curr.z, q_curr.y, q_curr.w)

    nv_gd.update_imu(acc_target, gyro_target, q_target, 0.0)

    last_pos = curr_pos
    last_vel = curr_vel
    last_basis = curr_basis
