package us.ihmc.sensorProcessing.parameters;

public interface AvatarRobotSensorParameters
{
   public boolean useRosForTransformFromPoseToSensor();
   public String getSensorNameInSdf();
   public String getRosTopic();
   public String getPoseFrameForSdf();
   public String getBaseFrameForRosTransform();
   public String getEndFrameForRosTransform();
   public int getSensorId();
   public AvatarRobotVisionSensorType getSensorType();
}
