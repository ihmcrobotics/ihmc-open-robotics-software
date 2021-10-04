package us.ihmc.avatar.heightMap;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;

class HeightMapDataToVisualize
{
   private final TIntArrayList xCells = new TIntArrayList();
   private final TIntArrayList yCells = new TIntArrayList();
   private final TDoubleArrayList heights = new TDoubleArrayList();

   public TIntArrayList getXCells()
   {
      return xCells;
   }

   public TIntArrayList getYCells()
   {
      return yCells;
   }

   public TDoubleArrayList getHeights()
   {
      return heights;
   }

   public void clear()
   {
      xCells.clear();
      yCells.clear();
      heights.clear();
   }
}
