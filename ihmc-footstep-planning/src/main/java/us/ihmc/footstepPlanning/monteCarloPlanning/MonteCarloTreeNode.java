package us.ihmc.footstepPlanning.monteCarloPlanning;

import java.util.ArrayList;
import java.util.List;

public abstract class MonteCarloTreeNode
{
   private ArrayList<MonteCarloTreeNode> parents;
   private ArrayList<MonteCarloTreeNode> children;

   protected int id = 0;
   protected float upperConfidenceBound = 0;
   protected int visits = 0;
   protected float value = 0;
   protected int level = 0;

   public MonteCarloTreeNode(MonteCarloTreeNode parent, int id)
   {
      this.id = id;
      this.parents = new ArrayList<>();
      this.children = new ArrayList<>();
      this.level = 0;


      if (parent != null)
      {
         parents.add(parent);
         this.level = parent.getLevel() + 1;
      }
   }

   public void updateUpperConfidenceBound()
   {
      if (visits == 0)
      {
         upperConfidenceBound = Float.MAX_VALUE;
         return;
      }

      // total visits of all parents
      double totalParentVists = parents.stream().mapToInt(MonteCarloTreeNode::getVisits).sum();
      upperConfidenceBound = (value / visits) + (MonteCarloPlannerConstants.EXPLORATION_WEIGHT * (float) Math.sqrt(Math.log(totalParentVists) / visits));
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

   public ArrayList<MonteCarloTreeNode> getParents()
   {
      return parents;
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
