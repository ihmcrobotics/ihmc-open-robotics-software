package object_manipulation_msgs;

public interface SceneRegion extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/SceneRegion";
  static final java.lang.String _DEFINITION = "# Point cloud\nsensor_msgs/PointCloud2 cloud\n\n# Indices for the region of interest\nint32[] mask\n\n# One of the corresponding 2D images, if applicable\nsensor_msgs/Image image\n\n# The disparity image, if applicable\nsensor_msgs/Image disparity_image\n\n# Camera info for the camera that took the image\nsensor_msgs/CameraInfo cam_info\n\n# a 3D region of interest for grasp planning\ngeometry_msgs/PoseStamped  roi_box_pose\ngeometry_msgs/Vector3 \t   roi_box_dims\n";
  sensor_msgs.PointCloud2 getCloud();
  void setCloud(sensor_msgs.PointCloud2 value);
  int[] getMask();
  void setMask(int[] value);
  sensor_msgs.Image getImage();
  void setImage(sensor_msgs.Image value);
  sensor_msgs.Image getDisparityImage();
  void setDisparityImage(sensor_msgs.Image value);
  sensor_msgs.CameraInfo getCamInfo();
  void setCamInfo(sensor_msgs.CameraInfo value);
  geometry_msgs.PoseStamped getRoiBoxPose();
  void setRoiBoxPose(geometry_msgs.PoseStamped value);
  geometry_msgs.Vector3 getRoiBoxDims();
  void setRoiBoxDims(geometry_msgs.Vector3 value);
}
