extends Node3D

@onready var armor_0_node = $A0
@onready var armor_1_node = $A1
@onready var armor_2_node = $A2
@onready var armor_3_node = $A3

@export var rotation_speed : float = PI

@export_category("Movement Settings (m)")
@export var max_speed: float = 5.0       # 最大巡航速度 (m/s)
@export var acceleration: float = 10.0   # 加速度 (m/s^2)
@export var deceleration: float = 15.0   # 减速度 (m/s^2)

@export_category("Patrol Settings")
@export var point_a: Vector3 = Vector3(0, 0, 0)    # 起点 A
@export var point_b: Vector3 = Vector3(10, 0, 5)   # 终点 B
@export var wait_time: float = 1.0                 # 到达端点后的停顿时间(秒)

var current_velocity: Vector3 = Vector3.ZERO
var target_position: Vector3

# 状态控制变量
var is_waiting: bool = false
var wait_timer: float = 0.0

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
    target_position = point_b
    pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
    rotate_y(delta * rotation_speed)
    pass

func _physics_process(delta: float):
    # 1. 停顿状态处理
    if is_waiting:
        wait_timer -= delta
        if wait_timer <= 0.0:
            is_waiting = false
            # 切换目标点：如果在 A 则去 B，反之亦然
            target_position = point_b if target_position.is_equal_approx(point_a) else point_a
        return # 停顿期间不执行后续逻辑

    # 2. 计算距离与方向
    var distance_to_target = global_position.distance_to(target_position)

    # 3. 到达目标点判定 (0.05米 = 5厘米的容差)
    if distance_to_target < 0.05:
        global_position = target_position
        current_velocity = Vector3.ZERO

        # 触发停顿并重置计时器
        is_waiting = true
        wait_timer = wait_time
        return

    # 4. 梯形速度计算
    var direction = (target_position - global_position).normalized()
    var current_speed = current_velocity.length()

    # 刹车距离公式：d = (v^2) / (2a)
    var stopping_distance = (current_speed * current_speed) / (2.0 * deceleration)

    if distance_to_target <= stopping_distance:
        # 减速阶段
        current_speed = move_toward(current_speed, 0.0, deceleration * delta)
    else:
        # 加速/匀速阶段
        current_speed = move_toward(current_speed, max_speed, acceleration * delta)

    # 5. 更新速度与位置
    current_velocity = direction * current_speed
    global_position += current_velocity * delta
