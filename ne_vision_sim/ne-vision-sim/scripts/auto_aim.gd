extends Node

# ==========================================
# 节点引用 & 基础配置
# ==========================================
@onready var auto_aim_viewport = $AutoAimViewport
@onready var gimbal = $AutoAimViewport/Gimbal
@export var auto_aim_result_frame : TextureRect

# 面板中可调的额外 Yaw 角度（单位：度）
@export var extra_yaw_degrees: float = 0.0

# ==========================================
# PD 控制器参数
# ==========================================
@export_group("PD Tracking Controller")
@export var kp_yaw: float = 10.0
@export var kd_yaw: float = 0.5
@export var kp_pitch: float = 10.0
@export var kd_pitch: float = 0.5

# ==========================================
# 内部状态变量
# ==========================================
@onready var nv_gd = NeVisionGd.new()
var fps_label: Label
var _display_texture: ImageTexture
var _last_q_godot: Quaternion = Quaternion.IDENTITY

# ==========================================
# 调试 UI 控件引用
# ==========================================
var debug_panel: CanvasLayer
var manual_mode_checkbox: CheckBox
var manual_yaw_spinbox: SpinBox
var manual_pitch_spinbox: SpinBox
var state_display_label: Label

# 缓存当前的状态数据，用于在 UI 上显示
var debug_data := {
	"target_yaw": 0.0, "target_pitch": 0.0,
	"current_yaw": 0.0, "current_pitch": 0.0,
	"out_yaw": 0.0, "out_pitch": 0.0
}

# ==========================================
# 初始化
# ==========================================
func _ready() -> void:
	var config_path = ProjectSettings.globalize_path("res://gdextensions/ne_vision_gd/config/config_sim.yaml")
	nv_gd.start(config_path)

	# 1. 设置 FPS Label
	fps_label = Label.new()
	fps_label.name = "FPSLabel"
	fps_label.add_theme_color_override("font_color", Color.GREEN)
	fps_label.add_theme_font_size_override("font_size", 24)
	fps_label.position = Vector2(10, 10)
	
	if auto_aim_result_frame:
		auto_aim_result_frame.add_child(fps_label)
	else:
		add_child(fps_label)

	# 2. 动态创建调试 UI 窗口
	_create_debug_ui()

# ==========================================
# 帧更新 (处理图像 & UI)
# ==========================================
func _process(delta: float) -> void:
	# 1. 获取并处理图像
	if auto_aim_viewport:
		var img = auto_aim_viewport.get_texture().get_image()
		if img and not img.is_empty():
			nv_gd.updata_frame(img)
			nv_gd.update_robot_info('B', 20)
			nv_gd.get_visualize_frame(img)

	# 2. 更新 FPS
	if fps_label:
		fps_label.text = "FPS: %d" % Engine.get_frames_per_second()
		
	# 3. 更新调试面板的文本显示
	if state_display_label:
		state_display_label.text = """=== 实时状态监控 ===
【当前云台】
Yaw: %.2f°
Pitch: %.2f°
【目标预期】
Yaw: %.2f°
Pitch: %.2f°
【误差 (Error)】
Yaw_Err: %.2f°
Pitch_Err: %.2f°
【PD 控制输出 (角速度)】
Out_Yaw: %.2f rad/s
Out_Pitch: %.2f rad/s
""" % [
			rad_to_deg(debug_data.current_yaw), rad_to_deg(debug_data.current_pitch),
			rad_to_deg(debug_data.target_yaw), rad_to_deg(debug_data.target_pitch),
			rad_to_deg(angle_difference(debug_data.current_yaw, debug_data.target_yaw)),
			rad_to_deg(angle_difference(debug_data.current_pitch, debug_data.target_pitch)),
			debug_data.out_yaw, debug_data.out_pitch
		]

# ==========================================
# 物理帧更新 (控制逻辑核心)
# ==========================================
func _physics_process(delta: float) -> void:
	if not gimbal: return
	
	# 1. 获取 Godot 坐标系下的全局姿态四元数
	var q_godot: Quaternion = gimbal.global_transform.basis.get_rotation_quaternion()
	
	# 2. 坐标系转换 (Camera 面向 Gimbal 的 +X 轴。Godot: X前, Y上, Z右 -> FLU: X前, Y左, Z上)
	var q_target = Quaternion(q_godot.x, -q_godot.z, q_godot.y, q_godot.w)
	
	# 3. 计算角速度 (根据四元数微分)
	var q_diff_godot = _last_q_godot.inverse() * q_godot
	var gyro_godot = Vector3(q_diff_godot.x, q_diff_godot.y, q_diff_godot.z) * 2.0 / delta
	if q_diff_godot.w < 0.0:
		gyro_godot = -gyro_godot
		
	var gyro_target = Vector3(gyro_godot.x, -gyro_godot.z, gyro_godot.y)
	_last_q_godot = q_godot
	var acc_target: Vector3 = Vector3.ZERO

	# 4. 传入 C++ 扩展更新 IMU
	nv_gd.update_imu(acc_target, gyro_target, q_target, delta)

	# ==================== 自瞄控制逻辑 ====================
	
	var target_yaw: float = 0.0
	var target_pitch: float = 0.0
	var target_yaw_v: float = 0.0
	var target_pitch_v: float = 0.0
	
	var should_track = false

	# 判断是否开启手动调试模式
	if manual_mode_checkbox and manual_mode_checkbox.button_pressed:
		# ======= 手动调试模式 =======
		should_track = true
		target_yaw = deg_to_rad(manual_yaw_spinbox.value)
		target_pitch = deg_to_rad(manual_pitch_spinbox.value)
		target_yaw_v = 0.0   # 手动模式下目标静止
		target_pitch_v = 0.0
	else:
		# ======= 视觉 AI 模式 =======
		var result: Dictionary = nv_gd.get_result()
		# state: 0=STOP, 1=ERROR, 2=WARNING, 3=IDLE, 4=AIMING
		if result.get("state", 0) == 4:
			should_track = true
			# C++ 输出为 IMU 世界系(FLU右手系)绝对角度，转换回 Godot 坐标系
			# IMU yaw(绕Z轴) -> Godot rotation.y;  IMU pitch(绕Y轴) -> Godot rotation.z = -IMU.y
			target_yaw   = result["yaw"]
			target_pitch = -result["pitch"]
			target_yaw_v   = result["yaw_v"]
			target_pitch_v = -result["pitch_v"]
			# print("yaw", target_yaw, "pitch", target_pitch);

			# 加上面板中设置的额外偏移
			# target_yaw += deg_to_rad(extra_yaw_degrees)

	if should_track:
		var target_pos = Vector2(target_yaw, target_pitch)
		var target_vel = Vector2(target_yaw_v, target_pitch_v)

		# 组装当前状态
		var current_yaw = gimbal.global_rotation.y
		var current_pitch = gimbal.global_rotation.z
		var current_pos = Vector2(current_yaw, current_pitch)
		var current_vel = Vector2(gyro_godot.y, gyro_godot.z)

		# 运行 PD 控制器
		var control_output = track_trajectory_pd(target_pos, target_vel, current_pos, current_vel)

		# 作用于云台 (按角速度移动)
		gimbal.global_rotation.y += control_output.x * delta
		gimbal.global_rotation.z += control_output.y * delta

		# 限制俯仰角，防止万向节死锁 (Gimbal Lock)
		gimbal.global_rotation.z = clamp(gimbal.global_rotation.z, deg_to_rad(-85), deg_to_rad(85))

		# --- 缓存数据供 UI 显示 ---
		debug_data.target_yaw = target_yaw
		debug_data.target_pitch = target_pitch
		debug_data.current_yaw = current_yaw
		debug_data.current_pitch = current_pitch
		debug_data.out_yaw = control_output.x
		debug_data.out_pitch = control_output.y

# ==========================================
# 核心算法：轨迹跟踪 PD 控制器
# ==========================================
func track_trajectory_pd(target_pos: Vector2, target_vel: Vector2, current_pos: Vector2, current_vel: Vector2) -> Vector2:
	# ⚠️ 必须使用 angle_difference 计算最短角度误差
	var error_yaw = angle_difference(current_pos.x, target_pos.x)
	var error_pitch = angle_difference(current_pos.y, target_pos.y)
	
	var error_vel = target_vel - current_vel
	
	var control_yaw = kp_yaw * error_yaw + kd_yaw * error_vel.x
	var control_pitch = kp_pitch * error_pitch + kd_pitch * error_vel.y
	
	return Vector2(control_yaw, control_pitch)

# ==========================================
# 辅助函数：纯代码生成调试 UI
# ==========================================
func _create_debug_ui() -> void:
	debug_panel = CanvasLayer.new()
	debug_panel.layer = 100 # 确保 UI 在最顶层
	add_child(debug_panel)
	
	var panel_bg = PanelContainer.new()
	panel_bg.position = Vector2(10, 50)
	panel_bg.custom_minimum_size = Vector2(250, 300)
	debug_panel.add_child(panel_bg)
	
	var margin = MarginContainer.new()
	margin.add_theme_constant_override("margin_top", 10)
	margin.add_theme_constant_override("margin_left", 10)
	margin.add_theme_constant_override("margin_right", 10)
	margin.add_theme_constant_override("margin_bottom", 10)
	panel_bg.add_child(margin)
	
	var vbox = VBoxContainer.new()
	margin.add_child(vbox)
	
	# 标题
	var title = Label.new()
	title.text = "云台控制调试台"
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	vbox.add_child(title)
	vbox.add_child(HSeparator.new())
	
	# 手动模式复选框
	manual_mode_checkbox = CheckBox.new()
	manual_mode_checkbox.text = "启用手动目标覆盖"
	vbox.add_child(manual_mode_checkbox)
	
	# 手动 Yaw 控制
	var yaw_hbox = HBoxContainer.new()
	var yaw_lbl = Label.new()
	yaw_lbl.text = "手动 Yaw°:"
	manual_yaw_spinbox = SpinBox.new()
	manual_yaw_spinbox.min_value = -180.0
	manual_yaw_spinbox.max_value = 180.0
	manual_yaw_spinbox.step = 1.0
	yaw_hbox.add_child(yaw_lbl)
	yaw_hbox.add_child(manual_yaw_spinbox)
	vbox.add_child(yaw_hbox)
	
	# 手动 Pitch 控制
	var pitch_hbox = HBoxContainer.new()
	var pitch_lbl = Label.new()
	pitch_lbl.text = "手动 Pitch°:"
	manual_pitch_spinbox = SpinBox.new()
	manual_pitch_spinbox.min_value = -85.0
	manual_pitch_spinbox.max_value = 85.0
	manual_pitch_spinbox.step = 1.0
	pitch_hbox.add_child(pitch_lbl)
	pitch_hbox.add_child(manual_pitch_spinbox)
	vbox.add_child(pitch_hbox)
	
	vbox.add_child(HSeparator.new())
	
	# 状态展示 Label
	state_display_label = Label.new()
	state_display_label.text = "等待数据..."
	vbox.add_child(state_display_label)
