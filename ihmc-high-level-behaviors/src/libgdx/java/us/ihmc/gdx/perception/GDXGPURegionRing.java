package us.ihmc.gdx.perception;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Vector2D;

public class GDXGPURegionRing
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
