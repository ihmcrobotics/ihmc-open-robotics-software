package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;

import java.util.ArrayList;

public class MonteCarloTreeNode
{
   private final Agent agent;
   private World world;
   private MonteCarloTreeNode parent;
   private ArrayList<MonteCarloTreeNode> children;

   private float upperConfidenceBound = 0;
   private int visits = 0;
   private float value = 0;

   float exploration_weight = 2.0f;

   public MonteCarloTreeNode(Point2D state, MonteCarloTreeNode parent)
   {
      this.parent = parent;
      this.agent = new Agent(state);
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

   public Agent getAgentState()
   {
      return agent;
   }

   public World getWorldState()
   {
      return world;
   }

   public int getVisits()
   {
      return visits;
   }

   public float getValue()
   {
      return value;
   }

   public void setValue(float value)
   {
      this.value = value;
   }

   public void setVisits(int visits)
   {
      this.visits = visits;
   }

   public void setWorldState(World world)
   {
      this.world = world;
   }

   public MonteCarloTreeNode getParent()
   {
      return parent;
   }

}
