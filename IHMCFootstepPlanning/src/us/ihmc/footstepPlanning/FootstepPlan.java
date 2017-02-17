package us.ihmc.footstepPlanning;

import java.util.ArrayList;
import java.util.Collections;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepPlan
{
   private final ArrayList<SimpleFootstep> footsteps = new ArrayList<>();

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
         SimpleFootstep simpleFootstep = addFootstep(node.getRobotSide(), framePose);
         if (node.isPartialFoothold())
         {
            simpleFootstep.setFoothold(node.getPartialFootholdPolygon());
         }

         node = node.getParentNode();
      }

      reverse();
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

   public SimpleFootstep addFootstep(RobotSide robotSide, FramePose soleFramePose)
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
      footsteps.clear();
   }
}
