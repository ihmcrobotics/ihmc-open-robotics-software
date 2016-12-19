package us.ihmc.footstepPlanning.aStar;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import org.junit.Test;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = IntegrationCategory.EXCLUDE)
public class AStarPlanarRegionsPlannerTest
{
   private class Node
   {
      private final double cost;

      public Node(double cost)
      {
         this.cost = cost;
      }

      public double getCost()
      {
         return cost;
      }
   }

   private class NodeComparator implements Comparator<Node>
   {
      @Override
      public int compare(Node o1, Node o2)
      {
         double cost1 = o1.getCost();
         double cost2 = o2.getCost();
         return cost1 < cost2 ? -1 : 1;
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testPriorityQueue()
   {
      double[] expected = new double[] {1.0, 2.0, 6.0, 9.0};
      PriorityQueue<Node> nodes = new PriorityQueue<>(new NodeComparator());
      nodes.add(new Node(1.0));
      nodes.add(new Node(6.0));
      nodes.add(new Node(2.0));
      nodes.add(new Node(9.0));

      int count = 0;
      while (!nodes.isEmpty())
      {
         Node node = nodes.poll();
         assertEquals(expected[count++], node.getCost(), 1.0e-10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testFootstepGraph()
   {
      FootstepNode startNode = new FootstepNode(0.0, 0.0);
      FootstepNode goalNode = new FootstepNode(4.0, 0.0);
      FootstepGraph graph = new FootstepGraph(startNode);
      double transitionCost = 1.0;

      // assemble simple graph structure
      graph.checkAndSetEdge(new FootstepNode(0.0, 0.0), new FootstepNode(1.0, 1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(0.0, 0.0), new FootstepNode(1.0, 0.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(0.0, 0.0), new FootstepNode(1.0, -1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(1.0, 1.0), new FootstepNode(2.0, 1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(1.0, -1.0), new FootstepNode(2.0, -1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(2.0, 1.0), new FootstepNode(3.0, 1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(2.0, -1.0), new FootstepNode(3.0, 1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(3.0, 1.0), goalNode, transitionCost);
      assertEquals(graph.getCostFromStart(goalNode), 4.0 * transitionCost, 1.0e-10);

      // add new edge that makes goal cheaper and check goal cost
      graph.checkAndSetEdge(new FootstepNode(1.0, 0.0), new FootstepNode(3.0, 1.0), transitionCost);
      assertEquals(graph.getCostFromStart(goalNode), 3.0 * transitionCost, 1.0e-10);

      // add new edge that should have no effect
      graph.checkAndSetEdge(new FootstepNode(2.0, -1.0), goalNode, transitionCost);
      assertEquals(graph.getCostFromStart(goalNode), 3.0 * transitionCost, 1.0e-10);

      // change goal node, add edge, and check cost
      FootstepNode newGoalNode = new FootstepNode(5.0, 0.0);
      graph.checkAndSetEdge(goalNode, newGoalNode, transitionCost);
      assertEquals(graph.getCostFromStart(newGoalNode), 4.0 * transitionCost, 1.0e-10);

      // update edge cost to be negative and make sure path to goal was updated
      graph.checkAndSetEdge(startNode, new FootstepNode(2.0, 1.0), -2.0);
      assertEquals(graph.getCostFromStart(newGoalNode), 1.0 * transitionCost, 1.0e-10);

      // check that goal path matches expected
      List<FootstepNode> pathToGoal = graph.getPathFromStart(newGoalNode);
      List<FootstepNode> expectedPathToGoal = new ArrayList<>();
      expectedPathToGoal.add(startNode);
      expectedPathToGoal.add(new FootstepNode(2.0, 1.0));
      expectedPathToGoal.add(new FootstepNode(3.0, 1.0));
      expectedPathToGoal.add(goalNode);
      expectedPathToGoal.add(newGoalNode);
      assertEquals(pathToGoal.size(), expectedPathToGoal.size());
      for (int i = 0; i < expectedPathToGoal.size(); i++)
      {
         FootstepNode node = pathToGoal.get(i);
         FootstepNode expectedNode = expectedPathToGoal.get(i);
         assertEquals(node, expectedNode);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testFootstepNode()
   {
      double gridX = FootstepNode.gridSizeX;
      double gridY = FootstepNode.gridSizeY;
      FootstepNode node;

      node = new FootstepNode(gridX * 0.3, 0.0);
      assertEquals(0.0, node.getX(), 1.0e-10);
      assertEquals(0.0, node.getY(), 1.0e-10);
      int hash1 = node.hashCode();

      node = new FootstepNode(gridX * 0.1, -gridY * 0.2);
      assertEquals(0.0, node.getX(), 1.0e-10);
      assertEquals(0.0, node.getY(), 1.0e-10);
      int hash2 = node.hashCode();

      assertEquals(hash1, hash2);

      node = new FootstepNode(gridX * 0.8, 0.0);
      assertEquals(gridX, node.getX(), 1.0e-10);
      assertEquals(0.0, node.getY(), 1.0e-10);

      node = new FootstepNode(gridX * 3.8, -gridY * 8.1);
      assertEquals(4.0 * gridX, node.getX(), 1.0e-10);
      assertEquals(-8.0 * gridY, node.getY(), 1.0e-10);
   }
}
