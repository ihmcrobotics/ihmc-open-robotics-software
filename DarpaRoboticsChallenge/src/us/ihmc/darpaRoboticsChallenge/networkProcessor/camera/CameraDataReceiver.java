package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import geometry_msgs.Transform;
import us.ihmc.atlas.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.driving.DRCStereoListener;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.messages.controller.RobotPoseData;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorControllerStateHandler;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.ros.ROSNativeTransformTools;
import us.ihmc.graphics3DAdapter.camera.CompressedVideoDataServer;
import us.ihmc.graphics3DAdapter.camera.VideoPacket;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.utilities.kinematics.TimeStampedTransform3D;
import us.ihmc.utilities.net.NetStateListener;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;

public abstract class CameraDataReceiver
{
   private final RobotPoseBuffer robotPoseBuffer;
   private final CompressedVideoDataServer compressedVideoDataServer;
   private final ArrayList<DRCStereoListener> stereoListeners = new ArrayList<DRCStereoListener>();

   private final Point3d cameraPosition = new Point3d();
   private final Quat4d cameraOrientation = new Quat4d();
   private final Vector3d tempVector = new Vector3d();

   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final ROSNativeTransformTools rosTransformProvider;
   private Transform3D rosTransformFromHeadBaseToCamera = new Transform3D();

   private final Transform3D cameraPose;

   public CameraDataReceiver(RobotPoseBuffer robotPoseBuffer, VideoSettings videoSettings, final DRCNetworkProcessorNetworkingManager networkingManager,
                             PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      this.robotPoseBuffer = robotPoseBuffer;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.cameraPose = new Transform3D();
      final DRCNetworkProcessorControllerStateHandler controllerStateHandler = networkingManager.getControllerStateHandler();

      ObjectCommunicator videoConsumer = new ObjectCommunicator()
      {
         @Override
         public void consumeObject(Object object)
         {
            if (object instanceof VideoPacket)
            {
               controllerStateHandler.sendVideoPacket((VideoPacket) object);
            }
         }

         @Override
         public boolean isConnected()
         {
            return networkingManager.isConnected();
         }

         @Override
         public void close()
         {
            networkingManager.close();
         }

         @Override
         public void attachStateListener(NetStateListener stateListener)
         {
            networkingManager.attachStateListener(stateListener);
         }

         @Override
         public <T> void attachListener(Class<T> clazz, ObjectConsumer<T> listener)
         {
         }

         @Override
         public void connect() throws IOException
         {
         }
      };

      if (DRCConfigParameters.USE_ROS_FOR_MULTISENSE_TRANSFORMS)
      {
         rosTransformProvider = ROSNativeTransformTools.getInstance(DRCConfigParameters.ROS_MASTER_URI);
         rosTransformProvider.connect();
      }
      else
      {
         rosTransformProvider = null;
      }

      compressedVideoDataServer = new CompressedVideoDataServer(videoSettings, videoConsumer);
      networkingManager.getControllerCommandHandler().setVideoCommandListener(compressedVideoDataServer);

   }

   protected void updateLeftEyeImage(BufferedImage bufferedImage, long timeStamp, double fov)
   {
      for (int i = 0; i < stereoListeners.size(); i++)
      {
         stereoListeners.get(i).leftImage(bufferedImage, timeStamp, fov);
      }

      RobotPoseData robotPoseData = robotPoseBuffer.floorEntry(ppsTimestampOffsetProvider.ajustTimeStampToRobotClock(timeStamp));
      if (robotPoseData == null)
      {
         return;
      }

      if (DRCConfigParameters.USE_ROS_FOR_MULTISENSE_TRANSFORMS)
      {
         if ((rosTransformFromHeadBaseToCamera.getType() & Transform3D.IDENTITY) != 0)
         {
            getHeadToCameraTransform(timeStamp);
         }

         if ((rosTransformFromHeadBaseToCamera.getType() & Transform3D.ZERO) != 0)
         {
            rosTransformFromHeadBaseToCamera.setIdentity();
         }

         cameraPose.mul(robotPoseData.getCameraPose(), rosTransformFromHeadBaseToCamera);
      }
      else
      {
         cameraPose.set(robotPoseData.getCameraPose());
      }

      cameraPose.get(cameraOrientation, tempVector);
      cameraPosition.set(tempVector);
      compressedVideoDataServer.updateImage(bufferedImage, timeStamp, cameraPosition, cameraOrientation, fov, DRCConfigParameters.MULTISENSE_LEFT_CAMERA);
   }

   protected void updateFishEyeImage(BufferedImage bufferedImage, long timeStamp, double fov, int sourceId)
   {
      RobotPoseData robotPoseData = robotPoseBuffer.floorEntry(ppsTimestampOffsetProvider.ajustTimeStampToRobotClock(timeStamp));

      if (robotPoseData == null)
      {
         return;
      }

      robotPoseData.getCameraPose().get(cameraOrientation, tempVector);
      cameraPosition.set(tempVector);
      compressedVideoDataServer.updateImage(bufferedImage, timeStamp, cameraPosition, cameraOrientation, fov, sourceId);


   }

   protected void getHeadToCameraTransform(long rosTimestamp)
   {
      if (ppsTimestampOffsetProvider.offsetIsDetermined())
      {
         long robotTimestamp = ppsTimestampOffsetProvider.ajustTimeStampToRobotClock(rosTimestamp);
         TimeStampedTransform3D transformFromRos = rosTransformProvider.getTimeStampedTransform("/left_camera_optical_frame", "/head", rosTimestamp,
                                                      robotTimestamp);
         rosTransformFromHeadBaseToCamera = transformFromRos.getTransform3D();
      }
   }

   protected void updateRightEyeImage(BufferedImage bufferedImage, long timeStamp, double fov)
   {
      for (int i = 0; i < stereoListeners.size(); i++)
      {
         stereoListeners.get(i).rightImage(bufferedImage, timeStamp, fov);
      }
   }

   public void registerCameraListener(DRCStereoListener drcStereoListener)
   {
      stereoListeners.add(drcStereoListener);
   }

}


//~ Formatted by Jindent --- http://www.jindent.com
