package us.ihmc.sensorProcessing.parameters;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public class DRCRobotCameraParameters implements DRCRobotSensorParameters
{
   private final String rosCompressedTopicName;
   private final String rosCameraInfoTopicName;
   private final String cameraNameInSdf;
   private final String rosEndFrameName;
   private final String rosBaseFrameName;
   private final String sdfPoseFrameName;
   private final RobotSide robotSide;
   private final int cameraId;
   private RigidBodyTransform staticTransform;

   public DRCRobotCameraParameters(RobotSide robotSide, String cameraName, String rosTopic, String poseFrameName, String rosCameraInfoTopic,RigidBodyTransform staticTransform, int cameraId)
   {
      this(robotSide, cameraName, rosTopic, rosCameraInfoTopic, poseFrameName, null, null, cameraId);
      this.staticTransform = staticTransform;
   }
   
   public DRCRobotCameraParameters(RobotSide robotSide, String cameraName, String rosTopic, String poseFrameName, int cameraId)
   {
      this(robotSide, cameraName,rosTopic,null,poseFrameName,null,null,cameraId);
   }

   public DRCRobotCameraParameters(RobotSide robotSide, String cameraName, String rosTopic, String poseFrameName, String rosCameraInfoTopic, int cameraId)
   {
      this(robotSide, cameraName, rosTopic, rosCameraInfoTopic, poseFrameName, null, null, cameraId);
   }

   public DRCRobotCameraParameters(RobotSide robotSide, String cameraName, String rosCameraTopic, String rosInfoTopic, String handOffFrameName, String rosBaseFrameName,
         String rosEndFrameName, int cameraId)
   {
      this.rosCompressedTopicName = rosCameraTopic;
      this.rosCameraInfoTopicName = rosInfoTopic;
      this.cameraNameInSdf = cameraName;
      this.sdfPoseFrameName = handOffFrameName;
      this.rosBaseFrameName = rosBaseFrameName;
      this.rosEndFrameName = rosEndFrameName;
      this.cameraId = cameraId;
      this.robotSide = robotSide;
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

   public boolean useStaticTransformFromHeadFrameToSensor()
   {
      return staticTransform != null;
   }

   public RigidBodyTransform getStaticTransformFromHeadFrameToCameraFrame()
   {
      return staticTransform;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }
}
