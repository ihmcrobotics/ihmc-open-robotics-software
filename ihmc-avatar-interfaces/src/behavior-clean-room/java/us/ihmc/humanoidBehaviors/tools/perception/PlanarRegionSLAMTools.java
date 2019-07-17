package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionSLAMTools
{
   public static void detectBoundingBoxCollisions()
   {

   }

   public static boolean boundingBoxesIntersect(PlanarRegion a, PlanarRegion b)
   {
      Box3D boxA = GeometryTools.convertBoundingBoxToBox(a.getBoundingBox3dInWorld());
      Box3D boxB = GeometryTools.convertBoundingBoxToBox(b.getBoundingBox3dInWorld());

      GilbertJohnsonKeerthiCollisionDetector gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector();

      EuclidShape3DCollisionResult collisionResult = gjkCollisionDetector.evaluateCollision(boxA, boxB);

      return collisionResult.areShapesColliding();
   }
}
