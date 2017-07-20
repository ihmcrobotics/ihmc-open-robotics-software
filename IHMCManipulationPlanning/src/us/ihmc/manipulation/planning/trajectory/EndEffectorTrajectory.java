package us.ihmc.manipulation.planning.trajectory;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/*
 * 170627 Inho Lee.
 */

public abstract class EndEffectorTrajectory
{
   protected double trajectoryTime = 0;
   protected RobotSide robotSideOfEndEffector;
   protected HandTrajectoryMessage endEffectorTrajectoryMessage;

   public abstract Pose3D getEndEffectorPose(double time);

   public abstract HandTrajectoryMessage getEndEffectorTrajectoryMessage(ReferenceFrame midFeetFrame);

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public void setRobotSideOfEndEffector(RobotSide robotSide)
   {
      robotSideOfEndEffector = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSideOfEndEffector;
   }

   public RobotSide getAnotherRobotSide()
   {
      if (robotSideOfEndEffector == RobotSide.RIGHT)
         return RobotSide.LEFT;
      else
         return RobotSide.RIGHT;
   }
}
