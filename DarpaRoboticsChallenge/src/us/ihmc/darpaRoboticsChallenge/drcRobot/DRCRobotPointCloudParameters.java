package us.ihmc.darpaRoboticsChallenge.drcRobot;


public class DRCRobotPointCloudParameters implements DRCRobotSensorParameters
{
   private final String pointCloudSensorName;
   private final String pointCloudTopic;
   private final String pointCloudBaseFrameForRos;
   private final String pointCloudEndFrameForRos;
   private final String lidarToRosHandoffFrameInSdf;
   private final boolean useRosForTransformFromPoseToSensor;
   private final int pointCloudId;
   
   public DRCRobotPointCloudParameters(String pointCloudSensorName, String pointCloudTopic, String sdflinkNameForSensorPose, int stereoId)
   {
      this.pointCloudSensorName = pointCloudSensorName;
      this.pointCloudTopic = pointCloudTopic;
      this.pointCloudBaseFrameForRos = null;
      this.pointCloudEndFrameForRos = null;
      this.lidarToRosHandoffFrameInSdf = sdflinkNameForSensorPose;
      this.useRosForTransformFromPoseToSensor = false;
      this.pointCloudId = stereoId;
   }

   public DRCRobotPointCloudParameters(String pointCloudSensorName, String pointCloudTopic, String lidarToRosHandoffFrameInSdf, String pointCloudBaseFrameForRos, String pointCloudEndFrameForRos, int stereoId)
   {
      this.pointCloudSensorName = pointCloudSensorName;
      this.pointCloudTopic = pointCloudTopic;
      this.lidarToRosHandoffFrameInSdf = lidarToRosHandoffFrameInSdf;
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
      return lidarToRosHandoffFrameInSdf;
   }
}
