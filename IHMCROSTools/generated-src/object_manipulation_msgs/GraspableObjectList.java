package object_manipulation_msgs;

public interface GraspableObjectList extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/GraspableObjectList";
  static final java.lang.String _DEFINITION = "object_manipulation_msgs/GraspableObject[] graspable_objects\n\n#Information required for visualization\n\nsensor_msgs/Image image\nsensor_msgs/CameraInfo camera_info\n\n#Holds a single mesh for each recognized graspable object, an empty mesh otherwise\nshape_msgs/Mesh[] meshes\n\n#pose to transform the frame of the clusters/object poses into camera coordinates\ngeometry_msgs/Pose reference_to_camera\n";
  java.util.List<object_manipulation_msgs.GraspableObject> getGraspableObjects();
  void setGraspableObjects(java.util.List<object_manipulation_msgs.GraspableObject> value);
  sensor_msgs.Image getImage();
  void setImage(sensor_msgs.Image value);
  sensor_msgs.CameraInfo getCameraInfo();
  void setCameraInfo(sensor_msgs.CameraInfo value);
  java.util.List<shape_msgs.Mesh> getMeshes();
  void setMeshes(java.util.List<shape_msgs.Mesh> value);
  geometry_msgs.Pose getReferenceToCamera();
  void setReferenceToCamera(geometry_msgs.Pose value);
}
