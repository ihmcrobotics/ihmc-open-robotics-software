package us.ihmc.sensorProcessing.parameters;

import us.ihmc.robotics.robotSide.RobotSide;

public class RealSenseR200Parameters implements RGBDCameraParameters
{
   private static final RobotSide robotSide = RobotSide.RIGHT;
   private static final String rosTopic = "/camera/rgb/image_rect_color";
   private static final String rosCameraInfoTopic = "/camera/rgb/camera_info";
   private static final String pointCloudTopic = "/camera/detph/points";

   private static final String cameraBaseFrame = "/camera_link";
   private static final String rgbCameraEndFrame = "/camera_rgb_optical_frame";
   private static final String irCameraEndFrame = "/camera_depth_optical_frame";

   private static final int cameraID = 0;
   private static final int irID = 0;

   private static final String cameraFrameNameInSDF = ""; //TODO: "kinect2_rgb_sensor_kinect2_rgb_sensor_camera";
   private static final String cameraNameInSDF = ""; //TODO: "kinect2_rgb_sensor_camera";
   private static final String pointCloudSensorName = ""; //TODO: "kinect2_pointcloud_sensor";
   private static final String sdfLinkNameForPointCloudPose = ""; //TODO: "kinect2_pitch";

   private final DRCRobotCameraParameters realSenseCameraParameters;
   private final DRCRobotPointCloudParameters realSensePointCloudParameters;

   public RealSenseR200Parameters()
   {
      realSenseCameraParameters = new DRCRobotCameraParameters(robotSide, cameraNameInSDF, rosTopic, rosCameraInfoTopic, cameraFrameNameInSDF, cameraBaseFrame,
            rgbCameraEndFrame, cameraID);

      realSensePointCloudParameters = new DRCRobotPointCloudParameters(pointCloudSensorName, pointCloudTopic, sdfLinkNameForPointCloudPose, cameraBaseFrame,
            irCameraEndFrame, irID);
   }

   @Override
   public DRCRobotCameraParameters getCameraParameters()
   {
      return realSenseCameraParameters;
   }

   @Override
   public DRCRobotPointCloudParameters getPointCloudParameters()
   {
      return realSensePointCloudParameters;
   }
}
