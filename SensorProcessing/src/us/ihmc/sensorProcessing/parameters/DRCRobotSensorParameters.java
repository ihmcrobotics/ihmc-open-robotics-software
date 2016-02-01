package us.ihmc.sensorProcessing.parameters;

public interface DRCRobotSensorParameters
{
   public boolean useRosForTransformFromPoseToSensor();
   public String getSensorNameInSdf();
   public String getRosTopic();
   public String getPoseFrameForSdf();
   public String getBaseFrameForRosTransform();
   public String getEndFrameForRosTransform();
   public int getSensorId();
   public DRCRobotSensorType getSensorType();
}
