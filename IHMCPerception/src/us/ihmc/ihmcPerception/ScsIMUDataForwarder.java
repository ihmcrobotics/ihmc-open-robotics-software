package us.ihmc.ihmcPerception;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.communication.packets.PacketDestination;

/**
 * This is only forwarding head IMU data recieved from scs to the controller. The
 * idea is to fake the pipeline the multisense IMU data goes through when running
 * the sim with network.
 *
 * @author Georg
 *
 */
public class ScsIMUDataForwarder implements PacketConsumer<IMUPacket>
{
   private final PacketCommunicator packetCommunicator;

   public ScsIMUDataForwarder(PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
   }

   @Override
   public void receivedPacket(IMUPacket packet)
   {
      packet.setDestination(PacketDestination.CONTROLLER);
      packetCommunicator.send(packet);
   }
}
