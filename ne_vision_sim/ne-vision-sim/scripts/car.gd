extends Node3D

@onready var armor_0_node = $A0
@onready var armor_1_node = $A1
@onready var armor_2_node = $A2
@onready var armor_3_node = $A3

@export var rotation_speed : float = PI

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	rotate_y(delta * rotation_speed)
	pass
	
