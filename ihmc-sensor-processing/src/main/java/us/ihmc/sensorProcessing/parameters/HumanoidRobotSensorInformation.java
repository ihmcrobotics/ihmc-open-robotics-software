package us.ihmc.sensorProcessing.parameters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public interface HumanoidRobotSensorInformation extends AvatarRobotRosVisionSensorInformation, HumanoidForceSensorInformation, IMUSensorInformation
{
   public default RigidBodyTransformReadOnly getSteppingCameraTransform()
   {
      return new RigidBodyTransform();
   }

   public default ReferenceFrame getSteppingCameraFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getPelvisFrame();
   }
}
