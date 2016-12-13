package us.ihmc.footstepPlanning.graphSearch;

import java.util.Comparator;
import java.util.PriorityQueue;

import org.junit.Test;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.io.printing.PrintTools;

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
      PriorityQueue<Node> nodes = new PriorityQueue<>(new NodeComparator());

      nodes.add(new Node(1.0));
      nodes.add(new Node(6.0));
      nodes.add(new Node(2.0));
      nodes.add(new Node(9.0));

      while (!nodes.isEmpty())
      {
         Node node = nodes.poll();
         PrintTools.info("Got Node with cost " + node.getCost());
      }
   }
}
