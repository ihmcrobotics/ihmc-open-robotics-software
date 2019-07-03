package us.ihmc.ihmcPerception.camera;

import java.util.Objects;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.utilities.ros.RosMainNode;

public abstract class RosCameraReceiver
{
   static final boolean DEBUG = false;
   private final RigidBodyTransform staticTransform = new RigidBodyTransform();

   public RosCameraReceiver(final AvatarRobotCameraParameters cameraParameters, final RosMainNode rosMainNode, final CameraLogger logger,
                            final CameraDataReceiver cameraDataReceiver)
   {
      if (cameraParameters.useRosForTransformFromPoseToSensor())
      {
         Objects.requireNonNull(cameraDataReceiver.getHeadFrame());
         // Start request for transform
         ROSHeadTransformFrame cameraFrame = new ROSHeadTransformFrame(cameraDataReceiver.getHeadFrame(), rosMainNode, cameraParameters);
         cameraDataReceiver.setCameraFrame(cameraFrame);
         new Thread(cameraFrame, "IHMC-RosCameraReceiver").start();

      }
      else if(cameraParameters.useStaticTransformFromHeadFrameToSensor())
      {
         Objects.requireNonNull(cameraDataReceiver.getHeadFrame());
         staticTransform.set(cameraParameters.getStaticTransformFromHeadFrameToCameraFrame());
         ReferenceFrame headFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("headToCamera", cameraDataReceiver.getHeadFrame(), staticTransform);
         cameraDataReceiver.setCameraFrame(headFrame);
      }
      else
      {
         Objects.requireNonNull(cameraDataReceiver.getHeadFrame());
         cameraDataReceiver.setCameraFrame(cameraDataReceiver.getHeadFrame());
      }

      final RosCameraInfoSubscriber imageInfoSubscriber;
      if (cameraParameters.useIntrinsicParametersFromRos())
      {
         imageInfoSubscriber = new RosCameraInfoSubscriber(cameraParameters.getRosCameraInfoTopicName());
         rosMainNode.attachSubscriber(cameraParameters.getRosCameraInfoTopicName(), imageInfoSubscriber);
      }
      else
      {
         throw new RuntimeException("You really want to use intrinisic parameters from ROS");
      }

      final RobotSide robotSide = cameraParameters.getRobotSide();

      createImageSubscriber(robotSide, logger, cameraDataReceiver, imageInfoSubscriber, rosMainNode, cameraParameters);
   }

   protected abstract void createImageSubscriber(RobotSide robotSide, CameraLogger logger, CameraDataReceiver cameraDataReceiver, RosCameraInfoSubscriber imageInfoSubscriber,
         RosMainNode rosMainNode, AvatarRobotCameraParameters cameraParameters);

}
