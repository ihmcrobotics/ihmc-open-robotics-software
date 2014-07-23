package us.ihmc.darpaRoboticsChallenge.drcRobot;

public interface DRCRobotSensorParameters
{
   public boolean useRosForTransformFromPoseToSensor();
   public String getSensorNameInSdf();
   public String getRosTopic();
   public String getPoseFrameForSdf();
   public String getBaseFrameForRosTransform();
   public String getEndFrameForRosTransform();
   public int getSensorId();
}
