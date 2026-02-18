extends Node

@onready var auto_aim_viewport = $AutoAimViewport
@export var auto_aim_result_frame : TextureRect

@onready var nv_gd = NeVisionGd.new()

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	var config_path = ProjectSettings.globalize_path("res://gdextensions/ne_vision_gd/config/config_sim.yaml")
	nv_gd.start(config_path)
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	
	# 1. 从后台视口抓取图像
	var img = auto_aim_viewport.get_texture().get_image()
	
	nv_gd.updata_frame(img)
	nv_gd.get_visualize_frame(img)
	
	# 3. 将结果更新到 UI 上的 TextureRect
	auto_aim_result_frame.texture = ImageTexture.create_from_image(img)
	
	pass
