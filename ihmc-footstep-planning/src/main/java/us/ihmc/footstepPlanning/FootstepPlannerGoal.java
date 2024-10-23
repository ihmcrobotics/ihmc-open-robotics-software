package us.ihmc.footstepPlanning;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;

public class FootstepPlannerGoal
{
   private FramePose3D goalPoseBetweenFeet;
   private SideDependentList<FramePose3D> doubleFootstepGoal;

   private double distanceProximity;
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

      this.distanceProximity = other.distanceProximity;
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

   public SideDependentList<? extends Pose3DReadOnly> getDoubleFootstepGoal()
   {
      return doubleFootstepGoal;
   }

   public void setDoubleFootstepGoal(SideDependentList<FramePose3D> doubleFootstepGoal)
   {
      this.doubleFootstepGoal = doubleFootstepGoal;
   }

   public FootstepPlannerGoalType getFootstepPlannerGoalType()
   {
      return footstepPlannerGoalType;
   }

   public double getDistanceProximity()
   {
      return distanceProximity;
   }

   public double getYawProximity()
   {
      return yawProximity;
   }

   public void setDistanceProximity(double distanceProximity)
   {
      this.distanceProximity = distanceProximity;
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
