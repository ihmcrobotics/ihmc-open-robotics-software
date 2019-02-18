package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.TimeInterval;

import java.util.ArrayList;
import java.util.Collections;

public class FootstepPlan
{
   private final ArrayList<QuadrupedTimedOrientedStep> footsteps = new ArrayList<>();
   private FramePose3DReadOnly lowLevelPlanGoal = null;
   private final FramePose3D startPose = new FramePose3D();
   private final FramePose3D goalPose = new FramePose3D();

   public FootstepPlan()
   {
   }

   public void setStartPose(FramePose3DReadOnly startPose)
   {
      this.startPose.setIncludingFrame(startPose);
   }

   public void setGoalPose(FramePose3DReadOnly goalPose)
   {
      this.goalPose.setIncludingFrame(goalPose);
   }

   public void setLowLevelPlanGoal(FramePose3DReadOnly lowLevelPlanGoal)
   {
      this.lowLevelPlanGoal = lowLevelPlanGoal;
   }

   public int getNumberOfSteps()
   {
      return footsteps.size();
   }

   public QuadrupedTimedStep getFootstep(int footstepIndex)
   {
      return footsteps.get(footstepIndex);
   }

   public void addFootstep(QuadrupedTimedOrientedStep footstep)
   {
      footsteps.add(footstep);
   }

   public void addFootstepPlan(FootstepPlan other)
   {
      footsteps.addAll(other.footsteps);
   }

   public QuadrupedTimedOrientedStep addFootstep(RobotQuadrant robotQuadrant, FramePoint3D soleFramePoint, TimeInterval timeInterval)
   {
      QuadrupedTimedOrientedStep footstep = new QuadrupedTimedOrientedStep();
      footstep.setRobotQuadrant(robotQuadrant);
      footstep.setGoalPosition(soleFramePoint);
      footstep.setTimeInterval(timeInterval);
      footsteps.add(footstep);
      return footstep;
   }

   public void reverse()
   {
      Collections.reverse(footsteps);
   }

   public void clear()
   {
      lowLevelPlanGoal = null;
      footsteps.clear();
   }

   public void remove(int footstepIndex)
   {
      footsteps.remove(footstepIndex);
   }

   public boolean hasLowLevelPlanGoal()
   {
      return lowLevelPlanGoal != null;
   }

   public FramePose3DReadOnly getLowLevelPlanGoal()
   {
      if (hasLowLevelPlanGoal())
         lowLevelPlanGoal.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      return lowLevelPlanGoal;
   }

   public FramePose3DReadOnly getStartPose()
   {
      return startPose;
   }
}
