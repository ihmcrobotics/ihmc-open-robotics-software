package us.ihmc.darpaRoboticsChallenge.drcRobot;


public class DRCRobotPointCloudParamaters
{
   private final String pointCloudSensorName;
   private final String pointCloudTopic;
   private final String pointCloudBaseFrameForRos;
   private final String pointCloudEndFrameForRos;

   public DRCRobotPointCloudParamaters(String pointCloudSensorName, String pointCloudTopic, String pointCloudBaseFrameForRos, String pointCloudEndFrameForRos)
   {
      this.pointCloudSensorName = pointCloudSensorName;
      this.pointCloudTopic = pointCloudTopic;
      this.pointCloudBaseFrameForRos = pointCloudBaseFrameForRos;
      this.pointCloudEndFrameForRos = pointCloudEndFrameForRos;
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
}
