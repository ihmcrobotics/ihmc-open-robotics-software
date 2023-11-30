package us.ihmc.footstepPlanning.monteCarloPlanning;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

public abstract class MonteCarloTreeNode implements Comparable<MonteCarloTreeNode>
{
   private final ArrayList<MonteCarloTreeNode> parents;
   private final ArrayList<MonteCarloTreeNode> children;

   protected int id = 0;
   protected float upperConfidenceBound = 0;
   protected int visits = 0;
   protected float value = 0;
   protected int level = 0;

   public MonteCarloTreeNode(MonteCarloTreeNode parent, int id)
   {
      this.id = id;

      this.level = 0;
      this.parents = new ArrayList<>();
      this.children = new ArrayList<>();

      if (parent != null)
      {
         parents.add(parent);
         this.level = parent.getLevel() + 1;
      }
   }

   public void updateUpperConfidenceBound(double explorationAlpha)
   {
      if (visits == 0)
      {
         upperConfidenceBound = Float.MAX_VALUE;
         return;
      }

      double totalParentVists = parents.stream().mapToInt(MonteCarloTreeNode::getVisits).sum();
      upperConfidenceBound = (value / visits) + ((float) explorationAlpha * (float) Math.sqrt(Math.log(totalParentVists) / visits));
   }

   public ArrayList<?> getAvailableStates(MonteCarloPlanningWorld world)
   {
      return null;
   }

   public float getUpperConfidenceBound()
   {
      return upperConfidenceBound;
   }

   //public List<MonteCarloTreeNode> getChildren()
   //{
   //   return children;
   //}

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

   public void setLevel(int level)
   {
      this.level = level;
   }

   public Object getState()
   {
      return null;
   }

   public MonteCarloTreeNode getMaxQueueNode()
   {
      return children.stream().max(Comparator.comparing(MonteCarloTreeNode::getValue)).get();
   }

   public void addChild(MonteCarloTreeNode child)
   {
      children.add(child);
   }

   public void removeChild(MonteCarloTreeNode child)
   {
      children.remove(child);
   }

   public void addParent(MonteCarloTreeNode parent)
   {
      parents.add(parent);
   }

   public void removeParent(MonteCarloTreeNode parent)
   {
      parents.remove(parent);
   }

   public int compareTo(MonteCarloTreeNode other)
   {
      if (this.getValue() > other.getValue())
      {
         return 1;
      }
      else if (this.getValue() < other.getValue())
      {
         return -1;
      }
      else
      {
         return 0;
      }
   }

   public void sortChildren()
   {
      // sort the children by value from highest to lowest
      children.sort(Comparator.comparing(MonteCarloTreeNode::getValue).reversed());
   }

   public void prune(int maxNumberOfChildren)
   {
      if (children.size() >= maxNumberOfChildren)
      {
         ArrayList<MonteCarloTreeNode> prunedQueue = new ArrayList<>();
         //System.out.println("Retaining Children: ");
         for (int i = 0; i < maxNumberOfChildren; i++)
         {
            prunedQueue.add(children.get(i));
            //System.out.print(children.get(i).getValue() + ", ");
         }
         //System.out.println();
         children.clear();
         children.addAll(prunedQueue);
      }
   }
}
