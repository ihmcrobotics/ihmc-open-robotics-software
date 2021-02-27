package us.ihmc.sensorProcessing.parameters;

import us.ihmc.euclid.transform.RigidBodyTransform;

public interface HumanoidRobotSensorInformation extends AvatarRobotRosVisionSensorInformation, HumanoidForceSensorInformation, IMUSensorInformation
{
   public default RigidBodyTransform getSteppingCameraTransform()
   {
      return new RigidBodyTransform();
   }
}
