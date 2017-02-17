package us.ihmc.ihmcPerception.camera;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.utilities.ros.RosMainNode;

public abstract class RosCameraReceiver
{
   static final boolean DEBUG = false;
   private final RigidBodyTransform staticTransform = new RigidBodyTransform();

   public RosCameraReceiver(final DRCRobotCameraParameters cameraParameters, final RosMainNode rosMainNode, final CameraLogger logger,
         final CameraDataReceiver cameraDataReceiver)
   {
      if (cameraParameters.useRosForTransformFromPoseToSensor())
      {
         // Start request for transform
         ROSHeadTransformFrame cameraFrame = new ROSHeadTransformFrame(cameraDataReceiver.getHeadFrame(), rosMainNode, cameraParameters);
         cameraDataReceiver.setCameraFrame(cameraFrame);
         new Thread(cameraFrame).start();

      }
      else if(cameraParameters.useStaticTransformFromHeadFrameToSensor())
      {
         staticTransform.set(cameraParameters.getStaticTransformFromHeadFrameToCameraFrame());
         ReferenceFrame headFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("headToCamera", cameraDataReceiver.getHeadFrame(), staticTransform);
         cameraDataReceiver.setCameraFrame(headFrame);
      }
      else
      {
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
         RosMainNode rosMainNode, DRCRobotCameraParameters cameraParameters);

}
