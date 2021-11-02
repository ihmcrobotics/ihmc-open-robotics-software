package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.robotics.geometry.PlaneFitter;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapper.createTransformToMatchSurfaceNormalPreserveX;
import static us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionPolygonSnapper.setTranslationSettingZAndPreservingXAndY;

public class HeightMapPolygonSnapper
{
   private final List<Point3D> pointsInsidePolyon = new ArrayList<>();
   private final Plane3D bestFitPlane = new Plane3D();
   private final LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   private double rSquared;
   private final double areaPerCell;

   public HeightMapPolygonSnapper()
   {
      HeightMapParameters parameters = new HeightMapParameters();
      areaPerCell = MathTools.square(parameters.getGridResolutionXY());
   }

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

      double averageHeight = 0.0;
      for (int i = 0; i < pointsInsidePolyon.size(); i++)
      {
         averageHeight += pointsInsidePolyon.get(i).getZ();
      }
      averageHeight /= pointsInsidePolyon.size();

      double ssRes = 0.0;
      double ssTot = 0.0;
      for (int i = 0; i < pointsInsidePolyon.size(); i++)
      {
         double zHat = bestFitPlane.getZOnPlane(pointsInsidePolyon.get(i).getX(), pointsInsidePolyon.get(i).getY());

         ssRes += MathTools.square(zHat - pointsInsidePolyon.get(i).getZ());
         ssTot += MathTools.square(averageHeight - pointsInsidePolyon.get(i).getZ());
      }

      rSquared = 1.0 - ssRes / ssTot;
//      System.out.println("R-Sq: " + rSq);

      if (bestFitPlane.containsNaN())
      {
         return null;
      }

      RigidBodyTransform transformToReturn = createTransformToMatchSurfaceNormalPreserveX(bestFitPlane.getNormal());

      Point2DReadOnly centroid = polygonToSnap.getCentroid();
      double height = bestFitPlane.getZOnPlane(centroid.getX(), centroid.getY());

      setTranslationSettingZAndPreservingXAndY(new Point3D(centroid.getX(), centroid.getY(), height), transformToReturn);

      return transformToReturn;
   }

   public Plane3D getBestFitPlane()
   {
      return bestFitPlane;
   }

   public double getRSquared()
   {
      return rSquared;
   }

   public double getArea()
   {
      return pointsInsidePolyon.size() * areaPerCell;
   }
}
