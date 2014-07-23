package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;

import javax.media.j3d.Transform3D;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCSensorParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.messages.controller.RobotPoseData;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.ros.ROSNativeTransformTools;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.ros.RosImageSubscriber;
import us.ihmc.utilities.ros.RosMainNode;

public class RosCameraReceiver extends CameraDataReceiver
{
   private static final boolean DEBUG = false;
   private Transform3D rosTransformFromHeadBaseToCamera = new Transform3D();
   private Transform3D worldToCameraTransform = new Transform3D();
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final ROSNativeTransformTools rosTransformProvider;
   private final DRCRobotCameraParameters cameraParameters;

   public RosCameraReceiver(final DRCRobotCameraParameters cameraParameters, final RobotPoseBuffer robotPoseBuffer, final VideoSettings videoSettings,
         final RosMainNode rosMainNode, final DRCNetworkProcessorNetworkingManager networkingManager, final PPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         final CameraLogger logger, String sensorURI)
   {
      super(robotPoseBuffer, videoSettings, networkingManager, ppsTimestampOffsetProvider);

      this.cameraParameters = cameraParameters;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;

      if (cameraParameters.useRosForTransformFromPoseToSensor())
      {
         rosTransformProvider = ROSNativeTransformTools.getInstance(sensorURI);
         rosTransformProvider.connect();
      }
      else
      {
         rosTransformProvider = null;
      }
      
      RosImageSubscriber imageSubscriberSubscriber = new RosImageSubscriber()
      {
         @Override
         protected void imageReceived(long timeStamp, BufferedImage image)
         {
            if (DEBUG)
               System.out.println("Cam image received. \ntimestamp: " + timeStamp);

            if (cameraParameters.useRosForTransformFromPoseToSensor())
            {
               if ((rosTransformFromHeadBaseToCamera.getType() & Transform3D.IDENTITY) != 0)
               {
                  getFrameToCameraTransform(timeStamp);
               }

               if ((rosTransformFromHeadBaseToCamera.getType() & Transform3D.ZERO) != 0)
               {
                  rosTransformFromHeadBaseToCamera.setIdentity();
               }
            }

            if( logger != null ) {
               logger.log(image, timeStamp);
            }

            RobotPoseData robotPoseData = robotPoseBuffer.floorEntry(ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(timeStamp));
            if(robotPoseData != null)
            {
               worldToCameraTransform.mul(robotPoseData.getCameraPoses()[0],rosTransformFromHeadBaseToCamera);
               updateLeftEyeImage(worldToCameraTransform, image, timeStamp, DRCSensorParameters.DUMMY_FILED_OF_VIEW);
            }
         }
      };
      rosMainNode.attachSubscriber(cameraParameters.getRosTopic(), imageSubscriberSubscriber);
   }

     
   private void getFrameToCameraTransform(long rosTimestamp)
   {
      if (ppsTimestampOffsetProvider.offsetIsDetermined())
      {
         long robotTimestamp = ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(rosTimestamp);
         TimeStampedTransform3D transformFromRos = rosTransformProvider.getTimeStampedTransform(cameraParameters.getEndFrameForRosTransform(),
               cameraParameters.getBaseFrameForRosTransform(), rosTimestamp, robotTimestamp);
         rosTransformFromHeadBaseToCamera = transformFromRos.getTransform3D();
      }
   }
}
