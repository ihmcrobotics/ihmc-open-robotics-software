package us.ihmc.darpaRoboticsChallenge.drcRobot;

import us.ihmc.graphics3DAdapter.camera.VideoSettingsH264LowLatency;

public class DRCRobotCameraParameters implements DRCRobotSensorParameters
{
   private final String rosCompressedTopicName;
   private final String rosCameraInfoTopicName;
   private final String cameraNameInSdf;
   private final String rosEndFrameName;
   private final String rosBaseFrameName;
   private final String sdfPoseFrameName;
   private final int cameraId;
   private final VideoSettingsH264LowLatency videoSettings;

   public DRCRobotCameraParameters(String cameraName, String rosTopic, String poseFrameName, VideoSettingsH264LowLatency videoSettings, int cameraId)
   {
      this(cameraName,rosTopic,null,poseFrameName,null,null,videoSettings,cameraId);
   }

   public DRCRobotCameraParameters(String cameraName, String rosCameraTopic, String rosInfoTopic, String handOffFrameName, String rosBaseFrameName,
         String rosEndFrameName, VideoSettingsH264LowLatency videoSettings, int cameraId)
   {
      this.rosCompressedTopicName = rosCameraTopic;
      this.rosCameraInfoTopicName = rosInfoTopic;
      this.cameraNameInSdf = cameraName;
      this.sdfPoseFrameName = handOffFrameName;
      this.rosBaseFrameName = rosBaseFrameName;
      this.rosEndFrameName = rosEndFrameName;
      this.videoSettings = videoSettings;
      this.cameraId = cameraId;
   }

   public VideoSettingsH264LowLatency getVideoSettings()
   {
      return videoSettings;
   }

   public String getRosTopic()
   {
      return rosCompressedTopicName;
   }

   public String getRosCameraInfoTopicName()
   {
      return rosCameraInfoTopicName;
   }

   public String getSensorNameInSdf()
   {
      return cameraNameInSdf;
   }

   public int getSensorId()
   {
      return cameraId;
   }

   public boolean useIntrinsicParametersFromRos()
   {
      return (rosCameraInfoTopicName != null && rosCameraInfoTopicName != "");
   }

   public boolean useRosForTransformFromPoseToSensor()
   {
      return (rosBaseFrameName != null && rosBaseFrameName != "") && (rosEndFrameName != null && rosEndFrameName != "");
   }

   public String getBaseFrameForRosTransform()
   {
      return rosBaseFrameName;
   }

   public String getEndFrameForRosTransform()
   {
      return rosEndFrameName;
   }

   @Override
   public String getPoseFrameForSdf()
   {
      return sdfPoseFrameName;
   }

   @Override
   public DRCRobotSensorType getSensorType()
   {
      return DRCRobotSensorType.CAMERA;
   }
}
