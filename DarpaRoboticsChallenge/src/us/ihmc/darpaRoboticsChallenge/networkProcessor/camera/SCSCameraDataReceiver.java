package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.graphics3DAdapter.camera.LocalVideoPacket;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;

public class SCSCameraDataReceiver extends CameraDataReceiver implements ObjectConsumer<LocalVideoPacket>
{

   public SCSCameraDataReceiver(RobotPoseBuffer robotPoseBuffer, VideoSettings videoSettings, ObjectCommunicator scsCommunicator, DRCNetworkProcessorNetworkingManager networkingManager)
   {
      super(robotPoseBuffer, videoSettings, networkingManager);

      scsCommunicator.attachListener(LocalVideoPacket.class, this);
   }

   public void consumeObject(LocalVideoPacket object)
   {
      updateLeftEyeImage(object.getImage(), object.getTimeStamp(), object.getFieldOfView());
   }

}
