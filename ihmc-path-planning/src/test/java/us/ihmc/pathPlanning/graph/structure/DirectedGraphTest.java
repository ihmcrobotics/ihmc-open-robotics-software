package us.ihmc.pathPlanning.graph.structure;

import org.junit.jupiter.api.Test;
import us.ihmc.pathPlanning.graph.GridNode;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class DirectedGraphTest
{
   @Test
   public void testGraph()
   {
      GridNode startNode = new GridNode(0, 0);
      GridNode goalNode = new GridNode(4, 0);
      DirectedGraph<GridNode> graph = new DirectedGraph<>();
      graph.initialize(startNode);
      double transitionCost = 1.0;

      // assemble simple graph structure
      graph.checkAndSetEdge(new GridNode(0, 0), new GridNode(1, 1), transitionCost);
      graph.checkAndSetEdge(new GridNode(0, 0), new GridNode(1, 0), transitionCost);
      graph.checkAndSetEdge(new GridNode(0, 0), new GridNode(1, -1), transitionCost);
      graph.checkAndSetEdge(new GridNode(1, 1), new GridNode(2, 1), transitionCost);
      graph.checkAndSetEdge(new GridNode(1, -1), new GridNode(2, -1), transitionCost);
      graph.checkAndSetEdge(new GridNode(2, 1), new GridNode(3, 1), transitionCost);
      graph.checkAndSetEdge(new GridNode(2, -1), new GridNode(3, 1), transitionCost);
      graph.checkAndSetEdge(new GridNode(3, 1), goalNode, transitionCost);
      assertEquals(graph.getCostFromStart(goalNode), 4.0 * transitionCost, 1.0e-10);

      // add new edge that makes goal cheaper and check goal cost
      graph.checkAndSetEdge(new GridNode(1, 0), new GridNode(3, 1), transitionCost);
      assertEquals(graph.getCostFromStart(goalNode), 3.0 * transitionCost, 1.0e-10);

      // add new edge that should have no effect
      graph.checkAndSetEdge(new GridNode(2, -1), goalNode, transitionCost);
      assertEquals(graph.getCostFromStart(goalNode), 3.0 * transitionCost, 1.0e-10);

      // change goal node, add edge, and check cost
      GridNode newGoalNode = new GridNode(5, 0);
      graph.checkAndSetEdge(goalNode, newGoalNode, transitionCost);
      assertEquals(graph.getCostFromStart(newGoalNode), 4.0 * transitionCost, 1.0e-10);

      // update edge cost to be negative and make sure path to goal was updated
      graph.checkAndSetEdge(startNode, new GridNode(2, 1), -2.0);
      assertEquals(graph.getCostFromStart(newGoalNode), 1.0 * transitionCost, 1.0e-10);

      // check that goal path matches expected
      List<GridNode> pathToGoal = graph.getPathFromStart(newGoalNode);
      List<GridNode> expectedPathToGoal = new ArrayList<>();
      expectedPathToGoal.add(startNode);
      expectedPathToGoal.add(new GridNode(2, 1));
      expectedPathToGoal.add(new GridNode(3, 1));
      expectedPathToGoal.add(goalNode);
      expectedPathToGoal.add(newGoalNode);
      assertEquals(pathToGoal.size(), expectedPathToGoal.size());
      for (int i = 0; i < expectedPathToGoal.size(); i++)
      {
         GridNode node = pathToGoal.get(i);
         GridNode expectedNode = expectedPathToGoal.get(i);
         assertEquals(node, expectedNode);
      }
   }
}
