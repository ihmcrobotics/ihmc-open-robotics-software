package us.ihmc.darpaRoboticsChallenge.drcRobot;


public class DRCRobotPointCloudParamaters
{
   private final String pointCloudSensorName;
   private final String pointCloudTopic;
   private final String pointCloudBaseFrameForRos;
   private final String pointCloudEndFrameForRos;
   private final boolean useRosForTransformFromPoseToSensor;
   private final int pointCloudId;
   
   public DRCRobotPointCloudParamaters(String pointCloudSensorName, String pointCloudTopic, int stereoId)
   {
      this.pointCloudSensorName = pointCloudSensorName;
      this.pointCloudTopic = pointCloudTopic;
      this.pointCloudBaseFrameForRos = null;
      this.pointCloudEndFrameForRos = null;
      this.useRosForTransformFromPoseToSensor = false;
      this.pointCloudId = stereoId;
   }

   public DRCRobotPointCloudParamaters(String pointCloudSensorName, String pointCloudTopic, String pointCloudBaseFrameForRos, String pointCloudEndFrameForRos, int stereoId)
   {
      this.pointCloudSensorName = pointCloudSensorName;
      this.pointCloudTopic = pointCloudTopic;
      this.pointCloudBaseFrameForRos = pointCloudBaseFrameForRos;
      this.pointCloudEndFrameForRos = pointCloudEndFrameForRos;
      this.useRosForTransformFromPoseToSensor = true;
      this.pointCloudId = stereoId;
   }
   
   public String getPointCloudSensorNameInSdf()
   {
      return pointCloudSensorName;
   }

   public String getPointCloudTopic()
   {
      return pointCloudTopic;
   }

   public String getPointCloudBaseFrameForRos()
   {
      return pointCloudBaseFrameForRos;
   }

   public String getPointCloudEndFrameForRos()
   {
      return pointCloudEndFrameForRos;
   }
   
   public int getPointCloudId()
   {
      return pointCloudId;
   }

   public boolean useRosForTransformFromPoseToSensor()
   {
      return useRosForTransformFromPoseToSensor;
   }
}
