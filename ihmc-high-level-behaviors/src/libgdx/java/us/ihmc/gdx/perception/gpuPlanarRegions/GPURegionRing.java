package us.ihmc.gdx.perception.gpuPlanarRegions;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Vector2D;

public class GPURegionRing
{
   private final RecyclingArrayList<Vector2D> boundaryIndices = new RecyclingArrayList<>(Vector2D::new);

   public void reset()
   {
      boundaryIndices.clear();
   }

   public RecyclingArrayList<Vector2D> getBoundaryIndices()
   {
      return boundaryIndices;
   }
}
