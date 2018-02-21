package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public class WalkToGoalBehaviorPacket extends Packet<WalkToGoalBehaviorPacket>
{
   public static final byte WALK_TO_GOAL_ACTION_FIND_PATH = 0;
   public static final byte WALK_TO_GOAL_ACTION_EXECUTE = 1;
   public static final byte WALK_TO_GOAL_ACTION_EXECUTE_UNKNOWN = 2;
   public static final byte WALK_TO_GOAL_ACTION_STOP = 3;

   public static final byte ROBOT_SIDE_LEFT = 0;
   public static final byte ROBOT_SIDE_RIGHT = 1;

   public byte walkToGoalAction;
   public double xGoal;
   public double yGoal;
   public double thetaGoal;

   public byte goalRobotSide;

   public WalkToGoalBehaviorPacket()
   {
      // for serialization
   }

   @Override
   public void set(WalkToGoalBehaviorPacket other)
   {
      walkToGoalAction = other.walkToGoalAction;
      xGoal = other.xGoal;
      yGoal = other.yGoal;
      thetaGoal = other.thetaGoal;
      goalRobotSide = other.goalRobotSide;
      setPacketInformation(other);
   }

   public byte getGoalRobotSide()
   {
      return goalRobotSide;
   }

   @Override
   public boolean epsilonEquals(WalkToGoalBehaviorPacket other, double epsilon)
   {
      if (!MathTools.epsilonEquals(xGoal, other.xGoal, epsilon))
         return false;
      if (!MathTools.epsilonEquals(yGoal, other.yGoal, epsilon))
         return false;
      if (!MathTools.epsilonEquals(thetaGoal, other.thetaGoal, epsilon))
         return false;
      return this.goalRobotSide == other.goalRobotSide;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + "\n";

      ret += "x: " + xGoal + "\n";
      ret += "y: " + yGoal + "\n";
      ret += "theta: " + thetaGoal + "\n";
      ret += "side: " + RobotSide.fromByte(goalRobotSide).toString() + "\n";

      return ret;
   }
}
