package us.ihmc.gdx.perception;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;

public class GDXGPURegionRing
{
   private final RecyclingArrayList<Point3D> boundaryVertices = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<Vector2D> boundaryIndices = new RecyclingArrayList<>(Vector2D::new);
   private int id;

   public void reset(int id)
   {
      this.id = id;
      boundaryVertices.clear();
      boundaryIndices.clear();
   }

   public RecyclingArrayList<Point3D> getBoundaryVertices()
   {
      return boundaryVertices;
   }

   public RecyclingArrayList<Vector2D> getBoundaryIndices()
   {
      return boundaryIndices;
   }
}
