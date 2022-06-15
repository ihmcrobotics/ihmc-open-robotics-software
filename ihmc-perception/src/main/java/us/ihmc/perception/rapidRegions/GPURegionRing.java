package us.ihmc.perception.rapidRegions;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Vector2D;

public class GPURegionRing
{
   private final RecyclingArrayList<Vector2D> boundaryIndices = new RecyclingArrayList<>(Vector2D::new);
   private int index;

   public void reset()
   {
      boundaryIndices.clear();
   }

   public RecyclingArrayList<Vector2D> getBoundaryIndices()
   {
      return boundaryIndices;
   }

   public void setIndex(int index)
   {
      this.index = index;
   }

   public int getIndex()
   {
      return index;
   }
}
