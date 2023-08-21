package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;

import java.util.ArrayList;

public class MonteCarloTreeNode
{
   private final MonteCarloPlanningAgent agent;
   private MonteCarloPlanningWorld world;
   private MonteCarloTreeNode parent;
   private ArrayList<MonteCarloTreeNode> children;

   private int id = 0;
   private float upperConfidenceBound = 0;
   private int visits = 0;
   private float value = 0;

   float exploration_weight = 2.0f;

   public MonteCarloTreeNode(Point2D state, MonteCarloTreeNode parent, int id)
   {
      this.id = id;
      this.parent = parent;
      this.agent = new MonteCarloPlanningAgent(state);
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

   public MonteCarloPlanningAgent getAgentState()
   {
      return agent;
   }

   public MonteCarloPlanningWorld getWorldState()
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

   public void setWorldState(MonteCarloPlanningWorld world)
   {
      this.world = world;
   }

   public MonteCarloTreeNode getParent()
   {
      return parent;
   }

   public int getId()
   {
      return id;
   }

}
