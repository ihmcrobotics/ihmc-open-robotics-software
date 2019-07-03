package us.ihmc.sensorProcessing.parameters;


public class AvatarRobotPointCloudParameters implements AvatarRobotSensorParameters
{
   private final String pointCloudSensorName;
   private final String pointCloudTopic;
   private final String pointCloudBaseFrameForRos;
   private final String pointCloudEndFrameForRos;
   private final String poseFrameName;
   private final boolean useRosForTransformFromPoseToSensor;
   private final int pointCloudId;
   
   public AvatarRobotPointCloudParameters(String pointCloudSensorName, String pointCloudTopic, String sdflinkNameForSensorPose, int stereoId)
   {
      this.pointCloudSensorName = pointCloudSensorName;
      this.pointCloudTopic = pointCloudTopic;
      this.pointCloudBaseFrameForRos = null;
      this.pointCloudEndFrameForRos = null;
      this.poseFrameName = sdflinkNameForSensorPose;
      this.useRosForTransformFromPoseToSensor = false;
      this.pointCloudId = stereoId;
   }

   public AvatarRobotPointCloudParameters(String pointCloudSensorName, String pointCloudTopic, String lidarToRosHandoffFrameInSdf, String pointCloudBaseFrameForRos, String pointCloudEndFrameForRos, int stereoId)
   {
      this.pointCloudSensorName = pointCloudSensorName;
      this.pointCloudTopic = pointCloudTopic;
      this.poseFrameName = lidarToRosHandoffFrameInSdf;
      this.pointCloudBaseFrameForRos = pointCloudBaseFrameForRos;
      this.pointCloudEndFrameForRos = pointCloudEndFrameForRos;
      this.useRosForTransformFromPoseToSensor = true;
      this.pointCloudId = stereoId;
   }
   
   public String getSensorNameInSdf()
   {
      return pointCloudSensorName;
   }

   public String getRosTopic()
   {
      return pointCloudTopic;
   }

   public String getBaseFrameForRosTransform()
   {
      return pointCloudBaseFrameForRos;
   }

   public String getEndFrameForRosTransform()
   {
      return pointCloudEndFrameForRos;
   }
   
   public int getSensorId()
   {
      return pointCloudId;
   }

   public boolean useRosForTransformFromPoseToSensor()
   {
      return useRosForTransformFromPoseToSensor;
   }
   
   @Override
   public String getPoseFrameForSdf()
   {
      return poseFrameName;
   }

   @Override
   public AvatarRobotVisionSensorType getSensorType()
   {
      return AvatarRobotVisionSensorType.POINTCLOUD;
   }
}
