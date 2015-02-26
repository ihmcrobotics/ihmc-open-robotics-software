package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.communication.util.DRCSensorParameters;
import us.ihmc.ros.jni.wrapper.ROSNativeTransformTools;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosImageSubscriber;

public class RosCameraReceiver extends CameraDataReceiver
{
   public RosCameraReceiver(SDFFullRobotModelFactory fullRobotModelFactory, final DRCRobotCameraParameters cameraParameters,
         RobotConfigurationDataBuffer robotConfigurationDataBuffer, final RosMainNode rosMainNode, final PacketCommunicator packetCommunicator,
         final PPSTimestampOffsetProvider ppsTimestampOffsetProvider, final CameraLogger logger, URI sensorURI)
   {
      super(fullRobotModelFactory, cameraParameters.getPoseFrameForSdf(), robotConfigurationDataBuffer, packetCommunicator, ppsTimestampOffsetProvider);
      if (cameraParameters.useRosForTransformFromPoseToSensor())
      {
         ROSNativeTransformTools rosTransformProvider = ROSNativeTransformTools.getInstance(sensorURI);
         rosTransformProvider.connect();

         TimeStampedTransform3D transformFromRos = rosTransformProvider.getTimeStampedTransform(cameraParameters.getBaseFrameForRosTransform(),
               cameraParameters.getEndFrameForRosTransform(), 0, 0);
         if (transformFromRos != null)
         {
            RigidBodyTransform headToCameraTransform = transformFromRos.getTransform3D();
            System.out.println("Head to camera transform from ROS: ");
            System.out.println(headToCameraTransform);

            ReferenceFrame cameraFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("rosHeadToCameraFrame", getHeadFrame(),
                  headToCameraTransform);
            setCameraFrame(cameraFrame);

         }
         else
         {
            System.err.println("Cannot get head to camera transform from ROS");
         }

      }

      RosImageSubscriber imageSubscriberSubscriber = new RosImageSubscriber()
      {
         @Override
         protected void imageReceived(long timeStamp, BufferedImage image)
         {
            if (logger != null)
            {
               logger.log(image, timeStamp);
            }

            updateLeftEyeImage(image, timeStamp, DRCSensorParameters.DUMMY_FIELD_OF_VIEW);

         }
      };
      rosMainNode.attachSubscriber(cameraParameters.getRosTopic(), imageSubscriberSubscriber);
   }

}
