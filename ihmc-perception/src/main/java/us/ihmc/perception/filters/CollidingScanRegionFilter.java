package us.ihmc.perception.filters;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.depthData.CollisionShapeTester;
import us.ihmc.robotEnvironmentAwareness.communication.converters.ScanRegionFilter;
import us.ihmc.robotics.geometry.PlanarRegion;

public class CollidingScanRegionFilter implements ScanRegionFilter
{
   private final CollisionShapeTester collisionBoxNode;

   public CollidingScanRegionFilter(CollisionShapeTester collisionBoxNode)
   {
      this.collisionBoxNode = collisionBoxNode;
   }

   public void update()
   {
      collisionBoxNode.update();
   }

   @Override
   public boolean test(int index, PlanarRegion region)
   {
      Point3D origin = new Point3D();
      region.getOrigin(origin);
      if (collisionBoxNode.contains(origin))
         return false;

      for (int i = 0; i < region.getConcaveHullSize(); i++)
      {
         Point3D vertex = new Point3D();
         vertex.set(region.getConcaveHullVertex(i));
         region.transformFromLocalToWorld(vertex);

         if (collisionBoxNode.contains(vertex))
            return false;
      }

      return !collisionBoxNode.contains(region);
   }

   public CollisionShapeTester getCollisionBoxNode()
   {
      return collisionBoxNode;
   }
}
