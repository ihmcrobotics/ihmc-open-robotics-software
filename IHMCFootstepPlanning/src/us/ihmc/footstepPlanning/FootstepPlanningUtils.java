package us.ihmc.footstepPlanning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerNode;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FootstepPlanningUtils
{
   public static FootstepPlan createFootstepPlanFromEndNode(BipedalFootstepPlannerNode endNode)
   {
      FootstepPlan plan = new FootstepPlan();
      plan.clear();
      BipedalFootstepPlannerNode node = endNode;

      while (node != null)
      {
         RigidBodyTransform soleTransform = new RigidBodyTransform();
         node.getSoleTransform(soleTransform);

         FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), soleTransform);
         plan.addFootstep(node.getRobotSide(), framePose);

         node = node.getParentNode();
      }

      plan.reverse();
      return plan;
   }

   public static List<BipedalFootstepPlannerNode> createListOfNodesFromEndNode(BipedalFootstepPlannerNode endNode)
   {
      List<BipedalFootstepPlannerNode> nodeList = new ArrayList<>();
      BipedalFootstepPlannerNode nodeToAdd = endNode;

      while(nodeToAdd != null)
      {
         nodeList.add(nodeToAdd);
         nodeToAdd = nodeToAdd.getParentNode();
      }

      Collections.reverse(nodeList);

      return nodeList;
   }
}
