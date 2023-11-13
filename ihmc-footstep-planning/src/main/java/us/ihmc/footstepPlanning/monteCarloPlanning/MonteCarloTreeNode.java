package us.ihmc.footstepPlanning.monteCarloPlanning;

import java.util.ArrayList;
import java.util.List;

public abstract class MonteCarloTreeNode
{
   private MonteCarloTreeNode parent;
   private ArrayList<MonteCarloTreeNode> children;

   protected int id = 0;
   protected float upperConfidenceBound = 0;
   protected int visits = 0;
   protected float value = 0;
   protected int level = 0;

   public MonteCarloTreeNode(MonteCarloTreeNode parent, int id)
   {
      this.id = id;
      this.parent = parent;
      this.children = new ArrayList<>();
      this.level = 0;

      if (parent != null)
         this.level = parent.getLevel() + 1;
   }

   public void updateUpperConfidenceBound()
   {
      if (visits == 0)
      {
         upperConfidenceBound = Float.MAX_VALUE;
         return;
      }

      upperConfidenceBound = (value / visits) + (MonteCarloPlannerConstants.EXPLORATION_WEIGHT * (float) Math.sqrt(Math.log(parent.visits) / visits));
   }

   public ArrayList<?> getAvailableStates(MonteCarloPlanningWorld world)
   {
      return null;
   }

   public float getUpperConfidenceBound()
   {
      return upperConfidenceBound;
   }

   public List<MonteCarloTreeNode> getChildren()
   {
      return children;
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

   public void incrementVisits()
   {
      visits++;
   }

   public void addValue(float value)
   {
      this.value += value;
   }

   public MonteCarloTreeNode getParent()
   {
      return parent;
   }

   public int getId()
   {
      return id;
   }

   public int getLevel()
   {
      return level;
   }

   public Object getPosition()
   {
      return null;
   }
}
