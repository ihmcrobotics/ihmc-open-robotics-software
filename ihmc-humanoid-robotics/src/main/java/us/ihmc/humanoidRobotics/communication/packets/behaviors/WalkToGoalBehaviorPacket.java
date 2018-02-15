package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public class WalkToGoalBehaviorPacket extends Packet<WalkToGoalBehaviorPacket>
{
   public static enum WalkToGoalAction
   {
      FIND_PATH, EXECUTE, EXECUTE_UNKNOWN, STOP
   };

   public WalkToGoalAction action;
   public double xGoal;
   public double yGoal;
   public double thetaGoal;

   public RobotSide goalSide;

   public WalkToGoalBehaviorPacket()
   {
      // for serialization
   }

   @Override
   public void set(WalkToGoalBehaviorPacket other)
   {
      action = other.action;
      xGoal = other.xGoal;
      yGoal = other.yGoal;
      thetaGoal = other.thetaGoal;
      goalSide = other.goalSide;
      setPacketInformation(other);
   }

   public double[] getGoalPosition()
   {
      return new double[] {xGoal, yGoal, thetaGoal};
   }

   public RobotSide getGoalSide()
   {
      return goalSide;
   }

   @Override
   public boolean epsilonEquals(WalkToGoalBehaviorPacket other, double epsilon)
   {
      boolean ret = true;
      double[] thisData = this.getGoalPosition();
      double[] otherData = other.getGoalPosition();
      for (int i = 0; (i < thisData.length && ret); i++)
      {
         ret = Math.abs(thisData[i] - otherData[i]) < epsilon;
      }
      ret &= this.goalSide == other.goalSide;
      return ret;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + "\n";

      ret += "x: " + xGoal + "\n";
      ret += "y: " + yGoal + "\n";
      ret += "theta: " + thetaGoal + "\n";
      ret += "side: " + goalSide.toString() + "\n";

      return ret;
   }
}
