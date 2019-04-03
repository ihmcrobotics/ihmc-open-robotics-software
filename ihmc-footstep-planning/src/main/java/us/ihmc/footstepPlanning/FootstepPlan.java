package us.ihmc.footstepPlanning;

import java.util.ArrayList;
import java.util.Collections;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlan
{
   private final ArrayList<SimpleFootstep> footsteps = new ArrayList<>();
   private FramePose3DReadOnly lowLevelPlanGoal = null;

   public FootstepPlan()
   {
   }

   public void setLowLevelPlanGoal(FramePose3DReadOnly lowLevelPlanGoal)
   {
      this.lowLevelPlanGoal = lowLevelPlanGoal;
   }

   public int getNumberOfSteps()
   {
      return footsteps.size();
   }

   public SimpleFootstep getFootstep(int footstepIndex)
   {
      return footsteps.get(footstepIndex);
   }

   public void addFootstep(SimpleFootstep footstep)
   {
      footsteps.add(footstep);
   }

   public void addFootstepPlan(FootstepPlan other)
   {
      footsteps.addAll(other.footsteps);
   }

   public SimpleFootstep addFootstep(RobotSide robotSide, FramePose3D soleFramePose)
   {
      SimpleFootstep simpleFootstep = new SimpleFootstep(robotSide, soleFramePose);
      footsteps.add(simpleFootstep);
      return simpleFootstep;
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
}
