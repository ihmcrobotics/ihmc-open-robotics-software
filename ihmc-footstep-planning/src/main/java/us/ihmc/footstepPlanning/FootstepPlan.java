package us.ihmc.footstepPlanning;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Collections;

public class FootstepPlan
{
   private final ArrayList<Footstep> footsteps = new ArrayList<>();

   public FootstepPlan()
   {
   }

   public int getNumberOfSteps()
   {
      return footsteps.size();
   }

   public Footstep getFootstep(int footstepIndex)
   {
      return footsteps.get(footstepIndex);
   }

   public void addFootstep(Footstep footstep)
   {
      footsteps.add(footstep);
   }

   public Footstep addFootstep(RobotSide robotSide, FramePose3D soleFramePose)
   {
      Footstep simpleFootstep = new Footstep(robotSide, soleFramePose);
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
