package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandCollisionDetectedPacket extends Packet<HandCollisionDetectedPacket>
{
 public RobotSide robotSide;
 public int collisionSeverityLevelOneToThree;
   
   public HandCollisionDetectedPacket(RobotSide robotSide, int collisionSeverityLevelZeroToThree)
   {
      this.robotSide = robotSide;
      this.collisionSeverityLevelOneToThree = MathTools.clamp(collisionSeverityLevelZeroToThree, 1, 3);
   }
   
   public HandCollisionDetectedPacket()
   {
   }
   
   public RobotSide getRobotSide()
   {
      return robotSide;
   }
   
   public int getCollisionSeverityOneToThree()
   {
      return collisionSeverityLevelOneToThree;
   }
   
   @Override
   public boolean epsilonEquals(HandCollisionDetectedPacket other, double epsilon) 
   {
      boolean robotSideEquals = robotSide == other.robotSide;;
      boolean collisionSeverityEquals = collisionSeverityLevelOneToThree == other.getCollisionSeverityOneToThree();
      return robotSideEquals && collisionSeverityEquals;
   }
}
