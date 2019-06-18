package us.ihmc.sensorProcessing.parameters;

public class AvatarRobotLidarParameters implements AvatarRobotSensorParameters
{
   private final String lidarSensorName;
   private final String laserTopic;
   private final String quadTreeCloudTopic;
   private final String lidarSpindleJointName;
   private final String lidarSpindleJointTopic;
   private final String lidarBaseFrameForRos;
   private final String lidarEndFrameForRos;
   private final String poseFrameName;
   private final double lidarSpinVelocity;
   private final boolean useRosForTransformFromPoseToSensor;
   private final int lidarId;

   public AvatarRobotLidarParameters(boolean useRosForTransformFromPoseToSensor, String lidarSensorName, String laserTopic, String groundCloudTopic, String lidarSpindleJointName, String lidarSpindleJointTopic,
                                     String poseFrameName, String lidarBaseFrameForRos, String lidarEndFrameForRos, double lidar_spindle_velocity, int lidarId)
   {
      this.useRosForTransformFromPoseToSensor = useRosForTransformFromPoseToSensor;
      this.lidarSensorName = lidarSensorName;//"head_hokuyo_sensor"
      this.laserTopic = laserTopic;//multisense_namespace+"/lidar_scan"
      this.quadTreeCloudTopic = groundCloudTopic;
      this.lidarSpindleJointName = lidarSpindleJointName;//"hokuyo_joint";
      this.lidarSpindleJointTopic = lidarSpindleJointTopic;//multisense_namespace + "/joint_states";
      this.poseFrameName = poseFrameName;//"head";
      this.lidarBaseFrameForRos = lidarBaseFrameForRos;//multisense_namespace + "/head";
      this.lidarEndFrameForRos = lidarEndFrameForRos;//multisense_namespace + "/head_hokuyo_frame";
      this.lidarSpinVelocity = lidar_spindle_velocity;//lidar_spindle_velocity = 5.1;
      this.lidarId = lidarId;
   }

   public String getSensorNameInSdf()
   {
      return lidarSensorName;
   }

   public String getRosTopic()
   {
      return laserTopic;
   }
   
   public String getGroundCloudTopic()
   {
      return quadTreeCloudTopic;
   }

   public String getLidarSpindleJointName()
   {
      return lidarSpindleJointName;
   }

   public String getLidarSpindleJointTopic()
   {
      return lidarSpindleJointTopic;
   }

   public String getBaseFrameForRosTransform()
   {
      return lidarBaseFrameForRos;
   }

   public String getEndFrameForRosTransform()
   {
      return lidarEndFrameForRos;
   }

   public double getLidarSpindleVelocity()
   {
      return lidarSpinVelocity;
   }

   public int getSensorId()
   {
      return lidarId;
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
      return AvatarRobotVisionSensorType.LIDAR;
   }
}