extends Node3D

@export var material_blue : StandardMaterial3D
@export var material_red : StandardMaterial3D

@onready var light_right_node = $LightRight
@onready var light_left_node = $LightLeft

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	light_left_node.set_surface_override_material(0, material_blue)
	light_right_node.set_surface_override_material(0, material_blue)
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
