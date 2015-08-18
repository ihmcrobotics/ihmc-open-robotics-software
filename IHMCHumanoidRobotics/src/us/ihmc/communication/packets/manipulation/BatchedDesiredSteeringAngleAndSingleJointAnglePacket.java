package us.ihmc.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.wholebody.SingleJointAnglePacket;

public class BatchedDesiredSteeringAngleAndSingleJointAnglePacket extends Packet<BatchedDesiredSteeringAngleAndSingleJointAnglePacket>
{
   public DesiredSteeringAnglePacket desiredSteeringAnglePacket;
   public SingleJointAnglePacket singleJointAnglePacket;

   public BatchedDesiredSteeringAngleAndSingleJointAnglePacket()
   {
      
   }

   public DesiredSteeringAnglePacket getDesiredSteeringAnglePacket()
   {
      return desiredSteeringAnglePacket;
   }

   public SingleJointAnglePacket getSingleJointAnglePacket()
   {
      return singleJointAnglePacket;
   }

   public BatchedDesiredSteeringAngleAndSingleJointAnglePacket(DesiredSteeringAnglePacket desiredSteeringAnglePacket, SingleJointAnglePacket singleJointAnglePacket)
   {
      setDestination(PacketDestination.CONTROLLER);
      this.desiredSteeringAnglePacket = desiredSteeringAnglePacket;
      this.singleJointAnglePacket = singleJointAnglePacket;
   }

   @Override
   public boolean epsilonEquals(BatchedDesiredSteeringAngleAndSingleJointAnglePacket other, double epsilon)
   {
      return desiredSteeringAnglePacket.epsilonEquals(other.desiredSteeringAnglePacket, epsilon)
            && singleJointAnglePacket.epsilonEquals(other.singleJointAnglePacket, epsilon);
   }

}
