package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class PlanarRegionSLAMTools
{
   public static Map<PlanarRegion, List<PlanarRegion>> detectBoundingBoxCollisions(PlanarRegionsList map, PlanarRegionsList newData)
   {
      HashMap<PlanarRegion, List<PlanarRegion>> newDataCollisions = new HashMap<>();
      for (PlanarRegion planarRegion : map.getPlanarRegionsAsList())
      {
         ArrayList<PlanarRegion> localCollisions = new ArrayList<>();
         newDataCollisions.put(planarRegion, localCollisions);

         for (PlanarRegion newRegion : newData.getPlanarRegionsAsList())
         {
            if (boundingBoxesIntersect(planarRegion, newRegion))
            {
               localCollisions.add(newRegion);
            }
         }
      }
      return newDataCollisions;
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
