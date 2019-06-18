package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
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
   private final FramePose3D lowLevelPlanGoal = new FramePose3D();

   public FootstepPlan()
   {
   }

   public void setLowLevelPlanGoal(FramePose3DReadOnly lowLevelPlanGoal)
   {
      this.lowLevelPlanGoal.setIncludingFrame(lowLevelPlanGoal);
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
      return addFootstep(robotQuadrant, soleFramePoint, timeInterval.getStartTime(), timeInterval.getEndTime());
   }

   public QuadrupedTimedOrientedStep addFootstep(RobotQuadrant robotQuadrant, FramePoint3D soleFramePoint, double startTime, double endTime)
   {
      QuadrupedTimedOrientedStep footstep = new QuadrupedTimedOrientedStep();
      footstep.setRobotQuadrant(robotQuadrant);
      footstep.setGoalPosition(soleFramePoint);
      footstep.getTimeInterval().setInterval(startTime, endTime);
      footsteps.add(footstep);
      return footstep;
   }

   public void reverse()
   {
      Collections.reverse(footsteps);
   }

   public void clear()
   {
      lowLevelPlanGoal.setToNaN();
      footsteps.clear();
   }

   public void remove(int footstepIndex)
   {
      footsteps.remove(footstepIndex);
   }

   public FramePose3DReadOnly getLowLevelPlanGoal()
   {
      lowLevelPlanGoal.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      return lowLevelPlanGoal;
   }
}
