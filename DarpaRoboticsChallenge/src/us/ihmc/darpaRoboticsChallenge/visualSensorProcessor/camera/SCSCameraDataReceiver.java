package us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.camera;

import us.ihmc.darpaRoboticsChallenge.visualSensorProcessor.state.RobotPoseBuffer;
import us.ihmc.graphics3DAdapter.camera.LocalVideoPacket;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.utilities.net.KryoObjectServer;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;

public class SCSCameraDataReceiver extends CameraDataReceiver implements ObjectConsumer<LocalVideoPacket>
{

   public SCSCameraDataReceiver(RobotPoseBuffer robotPoseBuffer, VideoSettings videoSettings, ObjectCommunicator scsCommunicator, KryoObjectServer teamServer)
   {
      super(robotPoseBuffer, videoSettings, teamServer);

      scsCommunicator.attachListener(LocalVideoPacket.class, this);
   }

   public void consumeObject(LocalVideoPacket object)
   {
      updateImage(object.getImage(), object.getTimeStamp(), object.getFieldOfView());
   }

}
