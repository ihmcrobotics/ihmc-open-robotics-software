package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;

public class ObjectWeightPacket extends Packet<ObjectWeightPacket>
{
   public byte robotSide;
   public double weight;

   public ObjectWeightPacket()
   {

   }

   @Override
   public void set(ObjectWeightPacket other)
   {
      robotSide = other.robotSide;
      weight = other.weight;
      setPacketInformation(other);
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   public double getWeight()
   {
      return weight;
   }

   @Override
   public boolean epsilonEquals(ObjectWeightPacket other, double epsilon)
   {
      boolean sameSide = robotSide == other.getRobotSide();
      boolean sameWeight = weight == other.getWeight();
      return sameSide && sameWeight;
   }
}
