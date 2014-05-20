package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;

import javax.media.j3d.Transform3D;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCSensorParameters;
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
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final ROSNativeTransformTools rosTransformProvider;
   private final DRCRobotCameraParamaters cameraParameters;

   public RosCameraReceiver(final DRCRobotCameraParamaters cameraParameters, final RobotPoseBuffer robotPoseBuffer, final VideoSettings videoSettings,
         final RosMainNode rosMainNode, final DRCNetworkProcessorNetworkingManager networkingManager, PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      super(robotPoseBuffer, videoSettings, networkingManager, ppsTimestampOffsetProvider);

      this.cameraParameters = cameraParameters;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;

      if (cameraParameters.useRosToGenerateTransformFromBaseToCamera())
      {
         rosTransformProvider = ROSNativeTransformTools.getInstance(DRCConfigParameters.ROS_MASTER_URI);
         rosTransformProvider.connect();
      }
      else
      {
         rosTransformProvider = null;
      }

      //XXX: move the multiSense setup out of DRC
      //      setupMultisenseResolution(rosMainNode);

      RosImageSubscriber imageSubscriberSubscriber = new RosImageSubscriber()
      {
         @Override
         protected void imageReceived(long timeStamp, BufferedImage image)
         {
            if (DEBUG)
               System.out.println("Cam image received. \ntimestamp: " + timeStamp);

            if (cameraParameters.useRosToGenerateTransformFromBaseToCamera())
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

            updateLeftEyeImage(rosTransformFromHeadBaseToCamera, image, timeStamp, DRCSensorParameters.DUMMY_FILED_OF_VIEW); //fov should come from intrinsic packet
         }
      };
      rosMainNode.attachSubscriber(cameraParameters.getRosCompressedTopicName(), imageSubscriberSubscriber);
   }

   private void getFrameToCameraTransform(long rosTimestamp)
   {
      if (ppsTimestampOffsetProvider.offsetIsDetermined())
      {
         long robotTimestamp = ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(rosTimestamp);
         TimeStampedTransform3D transformFromRos = rosTransformProvider.getTimeStampedTransform(cameraParameters.getCameraFrameName(),
               cameraParameters.getLowerFrameNameForRosTransform(), rosTimestamp, robotTimestamp);//"/left_camera_frame","/head"
         rosTransformFromHeadBaseToCamera = transformFromRos.getTransform3D();
      }
   }
}
