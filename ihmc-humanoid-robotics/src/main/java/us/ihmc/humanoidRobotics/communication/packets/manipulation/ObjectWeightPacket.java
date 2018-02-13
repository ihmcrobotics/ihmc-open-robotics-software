package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public class ObjectWeightPacket extends Packet<ObjectWeightPacket>
{
   public RobotSide robotSide;
   public double weight;
   
   public ObjectWeightPacket()
   {
      
   }
   
   public ObjectWeightPacket(RobotSide robotSide, double weight)
   {
      this.robotSide = robotSide;
      this.weight = weight;
   }
   
   public RobotSide getRobotSide()
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
      boolean sameSide = robotSide.equals(other.getRobotSide());
      boolean sameWeight = weight == other.getWeight();
      return sameSide && sameWeight;
   }
}
