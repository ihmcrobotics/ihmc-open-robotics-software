package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.blackoutGenerators.CommunicationBlackoutSimulator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.BlackoutPacketConsumer;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.LocalVideoPacket;
import us.ihmc.communication.packets.sensing.CameraInformationPacket;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;

/**
 * 
 *  Generate simulated camera data and camera info packet from SCS, we use only left eye.
 */
public class SCSCameraDataReceiver extends CameraDataReceiver implements PacketConsumer<LocalVideoPacket>
{
   private final SCSCameraInfoReceiver scsCameraInfoReceiver;

   public SCSCameraDataReceiver(SDFFullRobotModelFactory fullRobotModelFactory, String sensorNameInSdf, RobotConfigurationDataBuffer robotConfigurationDataBuffer, PacketCommunicator scsSensorsCommunicator,
         KryoLocalPacketCommunicator outgoingSensorDataCommunicator, PPSTimestampOffsetProvider ppsTimestampOffsetProvider)
   {
      super(fullRobotModelFactory, sensorNameInSdf, robotConfigurationDataBuffer, outgoingSensorDataCommunicator, ppsTimestampOffsetProvider);
      
      scsSensorsCommunicator.attachListener(LocalVideoPacket.class, this);

      CameraLogger logger = DRCConfigParameters.LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;

      scsCameraInfoReceiver = new SCSCameraInfoReceiver(outgoingSensorDataCommunicator, logger);
      outgoingSensorDataCommunicator.attachListener(CameraInformationPacket.class, scsCameraInfoReceiver);
   }

   public void receivedPacket(LocalVideoPacket object)
   {
      if (DEBUG)
      {
         System.out.println(getClass().getName() + ": received local video packet!");
      }
      updateLeftEyeImage(object.getImage(), object.getTimeStamp(), object.getFieldOfView());
      scsCameraInfoReceiver.setIntrinsicPacket(object);
   }
}
