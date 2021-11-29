package us.ihmc.sensorProcessing.parameters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public interface HumanoidRobotSensorInformation extends AvatarRobotRosVisionSensorInformation, HumanoidForceSensorInformation, IMUSensorInformation
{
   public default RigidBodyTransform getSteppingCameraTransform()
   {
      return new RigidBodyTransform();
   }

   public default ReferenceFrame getSteppingCameraParentFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getPelvisFrame();
   }

   public default ReferenceFrame getSteppingCameraFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent("steppingCamera",
                                                                             getSteppingCameraParentFrame(referenceFrames),
                                                                             getSteppingCameraTransform());
   }

   public default RigidBodyTransform getObjectDetectionCameraTransform()
   {
      return new RigidBodyTransform();
   }

   public default ReferenceFrame getObjectDetectionCameraParentFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getChestFrame();
   }

   public default ReferenceFrame getObjectDetectionCameraFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent("objectDetectionCamera",
                                                                             getObjectDetectionCameraParentFrame(referenceFrames),
                                                                             getObjectDetectionCameraTransform());
   }

   public default RigidBodyTransform getHeadZED2CameraTransform()
   {
      return new RigidBodyTransform();
   }

   public default ReferenceFrame getHeadZED2CameraParentFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getChestFrame();
   }

   public default ReferenceFrame getHeadZED2CameraFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent("headZED2Camera",
                                                                             getHeadZED2CameraParentFrame(referenceFrames),
                                                                             getHeadZED2CameraTransform());
   }

   public default RigidBodyTransform getOusterLidarTransform()
   {
      return new RigidBodyTransform();
   }

   public default ReferenceFrame getOusterLidarParentFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getChestFrame();
   }

   public default ReferenceFrame getOusterLidarFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return ReferenceFrameTools.constructFrameWithChangingTransformToParent("ousterLidar",
                                                                             getOusterLidarParentFrame(referenceFrames),
                                                                             getOusterLidarTransform());
   }

   public default String getHeadCameraName()
   {
      return null;
   }
}
