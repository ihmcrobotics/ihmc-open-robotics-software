package us.ihmc.footstepPlanning.log;

import us.ihmc.footstepPlanning.bodyPath.BodyPathLatticePoint;

import java.util.ArrayList;
import java.util.List;

public class AStarBodyPathIterationData
{
   private BodyPathLatticePoint parentNode = null;
   private final List<BodyPathLatticePoint> childNodes  = new ArrayList<>();
   private double parentNodeHeight = Double.NaN;

   public void setParentNode(BodyPathLatticePoint parentNode)
   {
      this.parentNode = parentNode;
   }

   public List<BodyPathLatticePoint> getChildNodes()
   {
      return childNodes;
   }

   public void setParentNodeHeight(double parentNodeHeight)
   {
      this.parentNodeHeight = parentNodeHeight;
   }

   public double getParentNodeHeight()
   {
      return parentNodeHeight;
   }

   public BodyPathLatticePoint getParentNode()
   {
      return parentNode;
   }

   public void clear()
   {
      parentNode = null;
      childNodes.clear();
      parentNodeHeight = Double.NaN;
   }
}
