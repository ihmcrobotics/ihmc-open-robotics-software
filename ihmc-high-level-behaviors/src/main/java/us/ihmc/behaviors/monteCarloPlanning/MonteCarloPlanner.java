package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;

public class MonteCarloPlanner
{
   private int searchIterations = 100;
   private int simulationIterations = 50;

   private WorldState world;
   private AgentState agent;
   private MonteCarloTreeNode root;

   public MonteCarloPlanner(WorldState world, AgentState agent)
   {
      this.world = world;
      this.agent = agent;

      root = new MonteCarloTreeNode(agent.getPosition());
   }

   public Point2D plan()
   {
      for (int i = 0; i < searchIterations; i++)
      {
         updateTree(root);
      }

      float bestScore = 0;
      MonteCarloTreeNode bestNode = null;
      for (MonteCarloTreeNode node : root.getChildren())
      {
         node.updateUpperConfidenceBound();
         if (node.getUpperConfidenceBound() > bestScore)
         {
            bestScore = node.getUpperConfidenceBound();
            bestNode = node;
         }
      }

      root = bestNode;

      return bestNode.getAgentState().getPosition();
   }

   public void updateTree(MonteCarloTreeNode node)
   {
      root = node;
   }
}
