package us.ihmc.footstepPlanning;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlannerObjective
{
   private double horizonLength = Double.NaN;
   private double timeout = Double.NaN;
   private double bestEffortTimeout = Double.NaN;
   private final FramePose3D initialStanceFootPose = new FramePose3D();
   private RobotSide initialStanceFootSide = null;
   private FootstepPlannerGoal goal = null;

   public FootstepPlannerObjective()
   {
      initialStanceFootPose.setToNaN();
   }

   public boolean hasHorizonLength()
   {
      return !Double.isNaN(horizonLength);
   }

   public boolean hasTimeout()
   {
      return !Double.isNaN(timeout);
   }

   public boolean hasBestEffortTimeout()
   {
      return !Double.isNaN(bestEffortTimeout);
   }

   public boolean hasGoal()
   {
      return goal != null;
   }

   public boolean hasInitialStanceFootPose()
   {
      return !initialStanceFootPose.containsNaN();
   }

   public boolean hasInitialStanceFootSide()
   {
      return initialStanceFootSide != null;
   }

   public void setHorizonLength(double horizonLength)
   {
      this.horizonLength = horizonLength;
   }

   public void setTimeout(double timeout)
   {
      this.timeout = timeout;
   }

   public void setBestEffortTimeout(double timeout)
   {
      this.bestEffortTimeout = timeout;
   }

   public void setInitialStanceFootPose(FramePose3DReadOnly initialStanceFootPose)
   {
      this.initialStanceFootPose.set(initialStanceFootPose);
   }

   public void setInitialStanceFootSide(RobotSide initialStanceFootSide)
   {
      this.initialStanceFootSide = initialStanceFootSide;
   }

   public void setGoal(FootstepPlannerGoal goal)
   {
      this.goal = goal;
   }

   public FramePose3D getInitialStanceFootPose()
   {
      return initialStanceFootPose;
   }

   public RobotSide getInitialStanceFootSide()
   {
      return initialStanceFootSide;
   }

   public double getHorizonLength()
   {
      return horizonLength;
   }

   public double getTimeout()
   {
      return timeout;
   }

   public double getBestEffortTimeout()
   {
      return bestEffortTimeout;
   }

   public FootstepPlannerGoal getGoal()
   {
      return goal;
   }
}
