package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;

import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;

public class MinimalGraphIndicesHolder implements GraphIndicesHolder
{
   ArrayList<Integer> keyPoints = new ArrayList<Integer>();
   
   int leftPlotIndex = 0;
   int rightPlotIndex = 100;
   int inPoint = 50;
   int index = 60;
   int outPoint = 75;
   int maxIndex = 200;
   
   @Override
   public void tickLater(int i)
   {
      index = index + i;
   }

   @Override
   public void setRightPlotIndex(int newRightIndex)
   {
      this.rightPlotIndex = newRightIndex;
   }

   @Override
   public void setLeftPlotIndex(int newLeftIndex)
   {
      this.leftPlotIndex = newLeftIndex;
   }

   @Override
   public void setIndexLater(int newIndex)
   {
      this.index = newIndex;
   }

   @Override
   public int getRightPlotIndex()
   {
      return rightPlotIndex;
   }

   @Override
   public int getMaxIndex()
   {
      return maxIndex;
   }

   @Override
   public int getLeftPlotIndex()
   {
      return leftPlotIndex;
   }

   @Override
   public ArrayList<Integer> getKeyPoints()
   {
      return keyPoints;
   }

   @Override
   public int getIndex()
   {
      return index;
   }

   @Override
   public int getInPoint()
   {
      return inPoint;
   }
   
   @Override
   public int getOutPoint()
   {
      return outPoint;
   }

   @Override
   public boolean isIndexAtOutPoint()
   {
      return (getIndex() == getOutPoint());
   }
}