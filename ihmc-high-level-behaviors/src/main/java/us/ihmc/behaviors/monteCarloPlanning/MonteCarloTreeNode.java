package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class MonteCarloTreeNode
{
   private Point2D position;
   private MonteCarloTreeNode parent;
   private ArrayList<MonteCarloTreeNode> children;

   private int id = 0;
   private float upperConfidenceBound = 0;
   private int visits = 0;
   private float value = 0;

   public MonteCarloTreeNode(Point2DReadOnly state, MonteCarloTreeNode parent, int id)
   {
      this.position = new Point2D(state);
      this.id = id;
      this.parent = parent;
      children = new ArrayList<>();
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

   public Point2D getPosition()
   {
      return position;
   }

}
