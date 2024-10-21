package us.ihmc.sensorProcessing.parameters;

import us.ihmc.robotics.robotSide.RobotSide;

public class RealSenseR200Parameters implements RGBDCameraParameters
{
   private final AvatarRobotCameraParameters realSenseCameraParameters;
   private final AvatarRobotPointCloudParameters realSensePointCloudParameters;

   public RealSenseR200Parameters(String namespace)
   {
      this(namespace, namespace);
   }

   public RealSenseR200Parameters(String topicNamespace, String cameraName)
   {
      String rosTopic = "/" + topicNamespace + "/rgb/image_raw";
      String rosCameraInfoTopic = "/" + topicNamespace + "/rgb/camera_info";
      String pointCloudTopic = "/" + topicNamespace + "/depth/points";

      String cameraBaseFrame = "/" + topicNamespace + "_link";
      String rgbCameraEndFrame = "/" + topicNamespace + "_rgb_optical_frame";
      String irCameraEndFrame = "/" + topicNamespace + "_depth_optical_frame";

      String cameraFrameNameInSDF = cameraName + "_rgb_sensor_" + cameraName + "_rgb_sensor_camera";
      String cameraNameInSDF = cameraName + "_rgb_sensor_camera";
      String pointCloudSensorName = cameraName + "_pointcloud_sensor";
      String sdfLinkNameForPointCloudPose = "";

      realSenseCameraParameters = new AvatarRobotCameraParameters(RobotSide.RIGHT, cameraNameInSDF, rosTopic, rosCameraInfoTopic, cameraFrameNameInSDF,
                                                                  cameraBaseFrame, rgbCameraEndFrame, 0);

      realSensePointCloudParameters = new AvatarRobotPointCloudParameters(pointCloudSensorName, pointCloudTopic, sdfLinkNameForPointCloudPose, cameraBaseFrame,
                                                                          irCameraEndFrame, 0);
   }

   @Override
   public AvatarRobotCameraParameters getCameraParameters()
   {
      return realSenseCameraParameters;
   }

   @Override
   public AvatarRobotPointCloudParameters getPointCloudParameters()
   {
      return realSensePointCloudParameters;
   }
}
