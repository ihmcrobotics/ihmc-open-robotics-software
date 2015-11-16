package us.ihmc.ihmcPerception.camera;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModelFactory;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.LocalVideoPacket;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;

/**
 *  Generate simulated camera data and camera info packet from SCS, we use only left eye.
 */
public class SCSCameraDataReceiver extends CameraDataReceiver implements ObjectConsumer<LocalVideoPacket>
{
   private static final boolean DEBUG = false;
   private final RobotSide robotSide;
   
   public SCSCameraDataReceiver(RobotSide robotSide, SDFFullHumanoidRobotModelFactory fullRobotModelFactory, String sensorNameInSdf, RobotConfigurationDataBuffer robotConfigurationDataBuffer, ObjectCommunicator scsSensorsCommunicator,
         PacketCommunicator sensorSuitePacketCommunicator, PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      super(fullRobotModelFactory, sensorNameInSdf, robotConfigurationDataBuffer, new VideoPacketHandler(sensorSuitePacketCommunicator), ppsTimestampOffsetProvider);
      
      this.robotSide = robotSide;

      scsSensorsCommunicator.attachListener(LocalVideoPacket.class, this);

//      CameraLogger logger = DRCConfigParameters.LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;
   }

   public void consumeObject(LocalVideoPacket object)
   {
      if (DEBUG)
      {
         System.out.println(getClass().getName() + ": received local video packet!");
      }
      updateImage(robotSide, object.getImage(), object.getTimeStamp(), object.getIntrinsicParameters());
   }
}
