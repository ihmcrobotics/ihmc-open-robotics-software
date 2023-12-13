package us.ihmc.perception.rapidRegions;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class RapidRegionRing
{
   private final RecyclingArrayList<Vector2D> boundaryIndices = new RecyclingArrayList<>(Vector2D::new);
   private int index;
   private final ConvexPolygon2D convexPolygon = new ConvexPolygon2D();

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

   public void updateConvexPolygon()
   {
      convexPolygon.clear();
      for (Vector2D boundaryIndex : boundaryIndices)
      {
         convexPolygon.addVertex(boundaryIndex.getX(), boundaryIndex.getY());
      }
      convexPolygon.update();
   }

   public ConvexPolygon2D getConvexPolygon()
   {
      return convexPolygon;
   }
}
