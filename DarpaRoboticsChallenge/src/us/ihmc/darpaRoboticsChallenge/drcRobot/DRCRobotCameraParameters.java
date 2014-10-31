package us.ihmc.darpaRoboticsChallenge.drcRobot;


public class DRCRobotCameraParameters implements DRCRobotSensorParameters
{
   private final String rosCompressedTopicName;
   private final String rosCameraInfoTopicName;
   private final String cameraNameInSdf;
   private final String rosEndFrameName;
   private final String rosBaseFrameName;
   private final String sdfPoseFrameName;
   private final int cameraId;

   public DRCRobotCameraParameters(String cameraName, String rosTopic, String poseFrameName, int cameraId)
   {
      this(cameraName,rosTopic,null,poseFrameName,null,null,cameraId);
   }

   public DRCRobotCameraParameters(String cameraName, String rosTopic, String poseFrameName, String rosCameraInfoTopic, int cameraId)
   {
      this(cameraName, rosTopic, rosCameraInfoTopic, poseFrameName, null, null, cameraId);
   }

   public DRCRobotCameraParameters(String cameraName, String rosCameraTopic, String rosInfoTopic, String handOffFrameName, String rosBaseFrameName,
         String rosEndFrameName, int cameraId)
   {
      this.rosCompressedTopicName = rosCameraTopic;
      this.rosCameraInfoTopicName = rosInfoTopic;
      this.cameraNameInSdf = cameraName;
      this.sdfPoseFrameName = handOffFrameName;
      this.rosBaseFrameName = rosBaseFrameName;
      this.rosEndFrameName = rosEndFrameName;
      this.cameraId = cameraId;
   }
   
   @Override
   public String getRosTopic()
   {
      return rosCompressedTopicName;
   }

   public String getRosCameraInfoTopicName()
   {
      return rosCameraInfoTopicName;
   }

   @Override
   public String getSensorNameInSdf()
   {
      return cameraNameInSdf;
   }

   @Override
   public int getSensorId()
   {
      return cameraId;
   }

   public boolean useIntrinsicParametersFromRos()
   {
      return (rosCameraInfoTopicName != null && rosCameraInfoTopicName != "");
   }

   @Override
   public boolean useRosForTransformFromPoseToSensor()
   {
      return (rosBaseFrameName != null && rosBaseFrameName != "") && (rosEndFrameName != null && rosEndFrameName != "");
   }

   @Override
   public String getBaseFrameForRosTransform()
   {
      return rosBaseFrameName;
   }

   @Override
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
