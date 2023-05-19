package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;

import java.util.ArrayList;

public class MonteCarloTreeNode
{
   private final AgentState agent;
   private WorldState world;
   private MonteCarloTreeNode parent;
   private ArrayList<MonteCarloTreeNode> children;

   private float upperConfidenceBound = 0;
   private int visits = 0;
   private float value = 0;

   float exploration_weight = 2.0f;

   public MonteCarloTreeNode(Point2D state)
   {
      this.agent = new AgentState(state);
      children = new ArrayList<>();
   }

   public void updateUpperConfidenceBound()
   {
      if (visits == 0)
      {
         upperConfidenceBound = Float.MAX_VALUE;
         return;
      }

      upperConfidenceBound = (value / visits) + (exploration_weight * (float) Math.sqrt(Math.log(parent.visits) / visits));
   }

   public float getUpperConfidenceBound()
   {
      return upperConfidenceBound;
   }

   public ArrayList<MonteCarloTreeNode> getChildren()
   {
      return children;
   }

   public AgentState getAgentState()
   {
      return agent;
   }

   public WorldState getWorldState()
   {
      return world;
   }

}
