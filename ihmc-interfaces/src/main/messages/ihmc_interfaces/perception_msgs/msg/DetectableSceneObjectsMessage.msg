# Gives the current state of the complete collection of detectable scene objects.
# Publishing all scene objects in one messages can simplify synchronization and
# reduce the complexity of logic in figuring out when objects are currently under
# consideraion.

# The current state of the complete collection of detectable scene objects
perception_msgs/DetectableSceneObjectMessage[<=5000] detectable_scene_objects