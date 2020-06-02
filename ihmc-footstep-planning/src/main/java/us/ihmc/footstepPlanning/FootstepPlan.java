package us.ihmc.footstepPlanning;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Collections;

public class FootstepPlan
{
   private final ArrayList<PlannedFootstep> footsteps = new ArrayList<>();

   public FootstepPlan()
   {
   }

   public int getNumberOfSteps()
   {
      return footsteps.size();
   }

   public PlannedFootstep getFootstep(int footstepIndex)
   {
      return footsteps.get(footstepIndex);
   }

   public void addFootstep(PlannedFootstep footstep)
   {
      footsteps.add(footstep);
   }

   public PlannedFootstep addFootstep(RobotSide robotSide, FramePose3D soleFramePose)
   {
      PlannedFootstep simpleFootstep = new PlannedFootstep(robotSide, soleFramePose);
      footsteps.add(simpleFootstep);
      return simpleFootstep;
   }

   public void reverse()
   {
      Collections.reverse(footsteps);
   }

   public void clear()
   {
      footsteps.clear();
   }

   public void remove(int footstepIndex)
   {
      footsteps.remove(footstepIndex);
   }
}
