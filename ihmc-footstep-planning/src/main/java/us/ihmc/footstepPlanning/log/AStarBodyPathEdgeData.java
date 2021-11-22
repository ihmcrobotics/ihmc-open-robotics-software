package us.ihmc.footstepPlanning.log;

import us.ihmc.footstepPlanning.bodyPath.BodyPathLatticePoint;

public class AStarBodyPathEdgeData
{
   private BodyPathLatticePoint parentNode = null;
   private BodyPathLatticePoint childNode = null;
   private double childSnapHeight = Double.NaN;
   private boolean solutionEdge = false;

   private final int capacity;
   private final long[] dataBuffer;

   public AStarBodyPathEdgeData(int capacity)
   {
      this.capacity = capacity;
      dataBuffer = new long[capacity];
   }

   public void clear()
   {
      parentNode = null;
      childNode = null;
      childSnapHeight = Double.NaN;
      solutionEdge = false;
   }

   public AStarBodyPathEdgeData getCopyAndClear()
   {
      AStarBodyPathEdgeData copy = new AStarBodyPathEdgeData(capacity);
      copy.parentNode = parentNode;
      copy.childNode = childNode;
      copy.childSnapHeight = childSnapHeight;
      copy.solutionEdge = solutionEdge;

      for (int i = 0; i < dataBuffer.length; i++)
      {
         copy.dataBuffer[i] = dataBuffer[i];
      }

      clear();
      return copy;
   }

   //////////////// GETTERS ////////////////

   public BodyPathLatticePoint getParentNode()
   {
      return parentNode;
   }

   public BodyPathLatticePoint getChildNode()
   {
      return childNode;
   }

   public double getChildSnapHeight()
   {
      return childSnapHeight;
   }

   public boolean isSolutionEdge()
   {
      return solutionEdge;
   }

   public long[] getDataBuffer()
   {
      return dataBuffer;
   }

   //////////////// SETTERS ////////////////

   public void setParentNode(BodyPathLatticePoint parentNode)
   {
      this.parentNode = parentNode;
   }

   public void setChildNode(BodyPathLatticePoint childNode)
   {
      this.childNode = childNode;
   }

   public void setChildSnapHeight(double childSnapHeight)
   {
      this.childSnapHeight = childSnapHeight;
   }

   public void setSolutionEdge(boolean solutionEdge)
   {
      this.solutionEdge = solutionEdge;
   }

   public void setData(int index, long data)
   {
      dataBuffer[index] = data;
   }
}
