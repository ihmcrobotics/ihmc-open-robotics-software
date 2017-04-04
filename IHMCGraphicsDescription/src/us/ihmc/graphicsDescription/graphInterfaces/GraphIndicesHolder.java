package us.ihmc.graphicsDescription.graphInterfaces;

import java.util.ArrayList;

public interface GraphIndicesHolder
{
   public abstract int getInPoint();

   public abstract int getOutPoint();

   public abstract int getIndex();

   public abstract int getLeftPlotIndex();

   public abstract int getRightPlotIndex();

   public abstract void tickLater(int i);

   public abstract void setIndexLater(int newIndex);

   public abstract void setLeftPlotIndex(int newLeftIndex);

   public abstract void setRightPlotIndex(int newRightIndex);

   public abstract int getMaxIndex();

   public abstract ArrayList<Integer> getKeyPoints();

   public abstract boolean isIndexAtOutPoint();
}
