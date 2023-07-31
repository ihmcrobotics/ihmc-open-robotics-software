package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.log.LogTools;

public class MonteCarloPlannerTools
{
   public static int getTotalNodes(MonteCarloTreeNode node)
   {
      if (node == null)
         return 0;

      int total = 1;
      for (MonteCarloTreeNode child : node.getChildren())
      {
         total += getTotalNodes(child);
      }

      return total;
   }

   public static void printTree(MonteCarloTreeNode node, int level)
   {
      System.out.printf("ID: %d\tLevel: %d\tNode: %s\tChildren: %d%n", node.getId(), level,
                        node.getAgentState().getPosition().toString(), node.getChildren().size());

      for (MonteCarloTreeNode child : node.getChildren())
      {
         printTree(child, level + 1);
      }
   }
}
