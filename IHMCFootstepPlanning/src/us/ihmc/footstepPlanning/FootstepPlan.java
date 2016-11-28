package us.ihmc.footstepPlanning;

import java.util.ArrayList;
import java.util.Collections;

import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlan
{
   public FootstepPlan()
   {
   }

   public FootstepPlan(BipedalFootstepPlannerNode endNode)
   {
      clear();
      BipedalFootstepPlannerNode node = endNode;

      while (node != null)
      {
         RigidBodyTransform soleTransform = new RigidBodyTransform();
         node.getSoleTransform(soleTransform);

         FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), soleTransform);
         addFootstep(node.getRobotSide(), framePose);

         node = node.getParentNode();
      }

      reverse();
   }

   private final ArrayList<SimpleFootstep> footsteps = new ArrayList<>();

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

   public void addFootstep(RobotSide robotSide, FramePose soleFramePose)
   {
      footsteps.add(new SimpleFootstep(robotSide, soleFramePose));
   }

   public void reverse()
   {
      Collections.reverse(footsteps);
   }

   public void clear()
   {
      footsteps.clear();
   }
}
