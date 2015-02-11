package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.net.URI;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.message.Time;

import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.sensing.RobotPoseData;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.communication.util.DRCSensorParameters;
import us.ihmc.darpaRoboticsChallenge.ros.RosRobotPosePublisher;
import us.ihmc.ros.jni.wrapper.ROSNativeTransformTools;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosPoseStampedPublisher;
import us.ihmc.utilities.ros.subscriber.RosImageSubscriber;

public class RosCameraReceiver extends CameraDataReceiver
{
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final ROSNativeTransformTools rosTransformProvider;
   private final DRCRobotCameraParameters cameraParameters;
   private RosPoseStampedPublisher robotPosePublisher;
   private final Vector3d position= new Vector3d();
   private final Quat4d orientation = new Quat4d();

   public RosCameraReceiver(final DRCRobotCameraParameters cameraParameters, final RobotPoseBuffer robotPoseBuffer,
         final RosMainNode rosMainNode, final PacketCommunicator packetCommunicator,
         final PPSTimestampOffsetProvider ppsTimestampOffsetProvider, final CameraLogger logger, URI sensorURI)
   {
      super(robotPoseBuffer, packetCommunicator, ppsTimestampOffsetProvider);

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
            
            RigidBodyTransform worldToCameraTransform = getCameraPose(robotPoseBuffer, timeStamp, cameraParameters.getSensorId());

            if(worldToCameraTransform != null)
            {
               if (cameraParameters.useRosForTransformFromPoseToSensor())
               {
                  RigidBodyTransform rosTransformFromHeadBaseToCamera = getFrameToCameraTransform(timeStamp);
                  if (rosTransformFromHeadBaseToCamera == null)
                  {
                     return;
                  }
                  worldToCameraTransform.multiply(rosTransformFromHeadBaseToCamera);
               }
               publishCameraWorldPoseForDebugging(worldToCameraTransform,timeStamp);
               updateLeftEyeImage(worldToCameraTransform, image, timeStamp, DRCSensorParameters.DUMMY_FIELD_OF_VIEW);
            }
         }
      };
      rosMainNode.attachSubscriber(cameraParameters.getRosTopic(), imageSubscriberSubscriber);
   }

   private RigidBodyTransform getCameraPose(RobotPoseBuffer robotPoseBuffer, long timeStamp, int sensorId)
   {
      RobotPoseData robotPoseData = robotPoseBuffer.floorEntry(ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(timeStamp));
      if (robotPoseData != null)
      {
         return robotPoseData.getCameraPose(sensorId);
      }
      return null;
   }

   private RigidBodyTransform getFrameToCameraTransform(long rosTimestamp)
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
   
   protected void publishCameraWorldPoseForDebugging(RigidBodyTransform transform, long timeStamp)
   {
      if(robotPosePublisher != null)
      {
         transform.get(orientation,position);
         robotPosePublisher.publish(RosRobotPosePublisher.WORLD_FRAME, position, orientation, Time.fromMillis(timeStamp));
      }
   }
}
