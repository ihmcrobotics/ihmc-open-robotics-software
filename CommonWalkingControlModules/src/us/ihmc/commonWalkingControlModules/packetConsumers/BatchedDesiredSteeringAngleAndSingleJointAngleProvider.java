package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.BatchedDesiredSteeringAngleAndSingleJointAnglePacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.DesiredSteeringAnglePacket;

public class BatchedDesiredSteeringAngleAndSingleJointAngleProvider implements PacketConsumer<BatchedDesiredSteeringAngleAndSingleJointAnglePacket>
{

   private final PacketConsumer<DesiredSteeringAnglePacket> desiredSteeringAngleConsumer;
   private final SingleJointPositionProvider singleJointPositionProvider;

   public BatchedDesiredSteeringAngleAndSingleJointAngleProvider(PacketConsumer<DesiredSteeringAnglePacket> desiredSteeringAngleConsumer,
         SingleJointPositionProvider singleJointPositionProvider)
   {
      this.desiredSteeringAngleConsumer = desiredSteeringAngleConsumer;
      this.singleJointPositionProvider = singleJointPositionProvider;
   }

   @Override
   public void receivedPacket(BatchedDesiredSteeringAngleAndSingleJointAnglePacket p)
   {
      singleJointPositionProvider.notifyWatchdogEventReceived();

      if(p.getDesiredSteeringAnglePacket() != null)
      {
         desiredSteeringAngleConsumer.receivedPacket(p.getDesiredSteeringAnglePacket());
      }
      if(p.getSingleJointAnglePacket() != null)
      {
         singleJointPositionProvider.receivedPacket(p.getSingleJointAnglePacket());
      }
      
   }
}
