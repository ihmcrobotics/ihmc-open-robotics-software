package us.ihmc.footstepPlanning;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepPlannerGoal
{
   private FramePose3D goalPoseBetweenFeet;
   private SideDependentList<SimpleFootstep> doubleFootstepGoal;

   private double xyProximity;
   private double yawProximity;

   private FootstepPlannerGoalType footstepPlannerGoalType;

   public void set(FootstepPlannerGoal other)
   {
      if (other.goalPoseBetweenFeet != null)
      {
         goalPoseBetweenFeet = new FramePose3D(other.goalPoseBetweenFeet);
      }

      if (other.doubleFootstepGoal != null)
      {
         doubleFootstepGoal = new SideDependentList<>();
         for (RobotSide robotSide : RobotSide.values)
            doubleFootstepGoal.put(robotSide, other.doubleFootstepGoal.get(robotSide));
      }

      this.xyProximity = other.xyProximity;
      this.yawProximity = other.yawProximity;

      if (other.footstepPlannerGoalType != null)
      {
         footstepPlannerGoalType = other.footstepPlannerGoalType;
      }
   }

   public FramePose3D getGoalPoseBetweenFeet()
   {
      return goalPoseBetweenFeet;
   }

   public void setGoalPoseBetweenFeet(FramePose3D goalPoseBetweenFeet)
   {
      this.goalPoseBetweenFeet = goalPoseBetweenFeet;
   }

   public SideDependentList<SimpleFootstep> getDoubleFootstepGoal()
   {
      return doubleFootstepGoal;
   }

   public void setDoubleFootstepGoal(SideDependentList<SimpleFootstep> doubleFootstepGoal)
   {
      this.doubleFootstepGoal = doubleFootstepGoal;
   }

   public FootstepPlannerGoalType getFootstepPlannerGoalType()
   {
      return footstepPlannerGoalType;
   }

   public double getXyProximity()
   {
      return xyProximity;
   }

   public double getYawProximity()
   {
      return yawProximity;
   }

   public void setXyProximity(double xyProximity)
   {
      this.xyProximity = xyProximity;
   }

   public void setYawProximity(double yawProximity)
   {
      this.yawProximity = yawProximity;
   }

   public void setFootstepPlannerGoalType(FootstepPlannerGoalType footstepPlannerGoalType)
   {
      this.footstepPlannerGoalType = footstepPlannerGoalType;
   }
}
