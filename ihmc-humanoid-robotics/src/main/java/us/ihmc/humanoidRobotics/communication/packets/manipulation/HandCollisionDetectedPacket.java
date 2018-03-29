package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;

public class HandCollisionDetectedPacket extends Packet<HandCollisionDetectedPacket>
{
   public static final byte ROBOT_SIDE_LEFT = 0;
   public static final byte ROBOT_SIDE_RIGHT = 1;

   public byte robotSide;
   public int collisionSeverityLevelOneToThree;

   public HandCollisionDetectedPacket()
   {
   }

   @Override
   public void set(HandCollisionDetectedPacket other)
   {
      robotSide = other.robotSide;
      collisionSeverityLevelOneToThree = other.collisionSeverityLevelOneToThree;
      setPacketInformation(other);
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   public int getCollisionSeverityLevelOneToThree()
   {
      return collisionSeverityLevelOneToThree;
   }

   @Override
   public boolean epsilonEquals(HandCollisionDetectedPacket other, double epsilon)
   {
      boolean robotSideEquals = robotSide == other.robotSide;
      boolean collisionSeverityEquals = collisionSeverityLevelOneToThree == other.getCollisionSeverityLevelOneToThree();
      return robotSideEquals && collisionSeverityEquals;
   }
}
