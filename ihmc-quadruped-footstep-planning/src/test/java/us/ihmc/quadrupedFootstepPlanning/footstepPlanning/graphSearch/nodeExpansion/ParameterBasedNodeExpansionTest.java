package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion;

import org.junit.Test;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.HashSet;

import static junit.framework.TestCase.fail;

public class ParameterBasedNodeExpansionTest
{
   @Test(timeout = 300000)
   public void testExpandNode()
   {
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);

      FootstepNode baseNode = new FootstepNode(RobotQuadrant.FRONT_LEFT, 0.5, 0.25, 0.5, -0.25, -0.5, 0.25, -0.5, -0.25);

      HashSet<FootstepNode> expandedNodes = expansion.expandNode(baseNode);
      fail();
   }
}
