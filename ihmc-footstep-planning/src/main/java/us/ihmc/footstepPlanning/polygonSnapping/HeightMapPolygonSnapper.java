package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.robotics.geometry.PlaneFitter;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapper.createTransformToMatchSurfaceNormalPreserveX;
import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapper.setTranslationSettingZAndPreservingXAndY;

public class HeightMapPolygonSnapper
{
   private final List<Point3D> pointsInsidePolyon = new ArrayList<>();
   private final Plane3D bestFitPlane = new Plane3D();
   private final PlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   public RigidBodyTransform snapPolygonToHeightMap(ConvexPolygon2DReadOnly polygonToSnap, HeightMapData heightMap)
   {
      pointsInsidePolyon.clear();
      bestFitPlane.setToNaN();

      int highestPointIndex = -1;
      double highestPointZ = Double.NEGATIVE_INFINITY;

      for (double x = polygonToSnap.getMinX(); x <= polygonToSnap.getMaxX(); x += heightMap.getGridResolutionXY())
      {
         for (double y = polygonToSnap.getMinY(); y <= polygonToSnap.getMaxY(); y += heightMap.getGridResolutionXY())
         {
            if (!polygonToSnap.isPointInside(x, y))
            {
               continue;
            }

            double height = heightMap.getHeightAt(x, y);
            if (Double.isNaN(height))
            {
               continue;
            }

            Point3D point = new Point3D(x, y, height);
            pointsInsidePolyon.add(point);

            if (point.getZ() > highestPointZ)
            {
               highestPointZ = point.getZ();
               highestPointIndex = pointsInsidePolyon.size() - 1;
            }
         }
      }

      if (pointsInsidePolyon.size() < 3)
      {
         return null;
      }

      planeFitter.fitPlaneToPoints(pointsInsidePolyon, bestFitPlane);
      RigidBodyTransform transformToReturn = createTransformToMatchSurfaceNormalPreserveX(bestFitPlane.getNormal());
      setTranslationSettingZAndPreservingXAndY(pointsInsidePolyon.get(highestPointIndex), transformToReturn);

      return transformToReturn;
   }
}
