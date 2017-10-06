package us.ihmc.footstepPlanning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public class FootstepPlanningUtils
{
   public static FootstepPlan createFootstepPlanFromEndNode(FootstepNode endNode)
   {
      FootstepPlan plan = new FootstepPlan();
      plan.clear();
      FootstepNode node = endNode;

      // TODO use graph to construct
//      while (node != null)
//      {
//         RigidBodyTransform soleTransform = new RigidBodyTransform();
//         BipedalFootstepPlannerNodeUtils.getSoleTransform(node, soleTransform);
//
//         FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), soleTransform);
//         plan.addFootstep(node.getRobotSide(), framePose);
//
//         node = node.getParentNode();
//      }

      plan.reverse();
      return plan;
   }

   public static List<FootstepNode> createListOfNodesFromEndNode(FootstepNode endNode)
   {
      List<FootstepNode> nodeList = new ArrayList<>();
      FootstepNode nodeToAdd = endNode;

      // TODO use graph to construct
//      while(nodeToAdd != null)
//      {
//         nodeList.add(nodeToAdd);
//         nodeToAdd = nodeToAdd.getParentNode();
//      }

      Collections.reverse(nodeList);

      return nodeList;
   }
}
