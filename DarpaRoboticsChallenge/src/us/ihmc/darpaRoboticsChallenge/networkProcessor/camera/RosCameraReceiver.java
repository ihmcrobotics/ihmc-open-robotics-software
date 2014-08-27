package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.net.URI;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.message.Time;

import us.ihmc.communication.networking.DRCSensorParameters;
import us.ihmc.communication.packets.sensing.RobotPoseData;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.ros.ROSNativeTransformTools;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotPosePublisher;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.ros.RosImageSubscriber;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosPoseStampedPublisher;

public class RosCameraReceiver extends CameraDataReceiver
{
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final ROSNativeTransformTools rosTransformProvider;
   private final DRCRobotCameraParameters cameraParameters;
   private RosPoseStampedPublisher robotPosePublisher;
   private Vector3d position= new Vector3d();
   private Quat4d orientation = new Quat4d();

   public RosCameraReceiver(final DRCRobotCameraParameters cameraParameters, final RobotPoseBuffer robotPoseBuffer, final VideoSettings videoSettings,
         final RosMainNode rosMainNode, final DRCNetworkProcessorNetworkingManager networkingManager,
         final PPSTimestampOffsetProvider ppsTimestampOffsetProvider, final CameraLogger logger, URI sensorURI)
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
            if (logger != null)
            {
               logger.log(image, timeStamp);
            }
            
            Transform3D worldToCameraTransform = getCameraPose(robotPoseBuffer, timeStamp, cameraParameters.getSensorId());

            if(worldToCameraTransform != null)
            {
               if (cameraParameters.useRosForTransformFromPoseToSensor())
               {
                  Transform3D rosTransformFromHeadBaseToCamera = getFrameToCameraTransform(timeStamp);
                  if (rosTransformFromHeadBaseToCamera == null)
                  {
                     return;
                  }
                  worldToCameraTransform.mul(rosTransformFromHeadBaseToCamera);
               }
               publishCameraWorldPoseForDebugging(worldToCameraTransform,timeStamp);
               updateLeftEyeImage(worldToCameraTransform, image, timeStamp, DRCSensorParameters.DUMMY_FILED_OF_VIEW);
            }
         }
      };
      rosMainNode.attachSubscriber(cameraParameters.getRosTopic(), imageSubscriberSubscriber);
   }

   private Transform3D getCameraPose(RobotPoseBuffer robotPoseBuffer, long timeStamp, int sensorId)
   {
      RobotPoseData robotPoseData = robotPoseBuffer.floorEntry(ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(timeStamp));
      if (robotPoseData != null)
      {
         return robotPoseData.getCameraPose(sensorId);
      }
      return null;
   }

   private Transform3D getFrameToCameraTransform(long rosTimestamp)
   {
      if (ppsTimestampOffsetProvider.offsetIsDetermined())
      {
         long robotTimestamp = ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(rosTimestamp);
         TimeStampedTransform3D transformFromRos = rosTransformProvider.getTimeStampedTransform(cameraParameters.getBaseFrameForRosTransform(),
               cameraParameters.getEndFrameForRosTransform(), rosTimestamp, robotTimestamp);
         if(transformFromRos != null)
         {
            return transformFromRos.getTransform3D();
         }
      }
      return null;
   }
   
   public void setRobotPosePublisher(RosRobotPosePublisher robotPosePublisher)
   {
      this.robotPosePublisher = robotPosePublisher.createPosePublisher("cameraSensorPoses/multiSenseCamera" + cameraParameters.getSensorId());
   }
   
   protected void publishCameraWorldPoseForDebugging(Transform3D transform, long timeStamp)
   {
      if(robotPosePublisher != null)
      {
         transform.get(orientation,position);
         robotPosePublisher.publish(RosRobotPosePublisher.WORLD_FRAME, position, orientation, Time.fromMillis(timeStamp));
      }
   }
}
