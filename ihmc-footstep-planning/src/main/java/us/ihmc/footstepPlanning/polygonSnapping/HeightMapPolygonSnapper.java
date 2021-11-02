package us.ihmc.footstepPlanning.polygonSnapping;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
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

      int minMaxIndex = HeightMapTools.toIndex(heightMap.getGridSizeXY(), heightMap.getGridResolutionXY(), 0);
      int minIndexX = HeightMapTools.toIndex(polygonToSnap.getMinX(), heightMap.getGridResolutionXY(), minMaxIndex);
      int maxIndexX = HeightMapTools.toIndex(polygonToSnap.getMaxX(), heightMap.getGridResolutionXY(), minMaxIndex);
      int minIndexY = HeightMapTools.toIndex(polygonToSnap.getMinY(), heightMap.getGridResolutionXY(), minMaxIndex);
      int maxIndexY = HeightMapTools.toIndex(polygonToSnap.getMaxY(), heightMap.getGridResolutionXY(), minMaxIndex);
      double averageHeight = 0.0;

      for (int i = minIndexX; i <= maxIndexX; i++)
      {
         for (int j = minIndexY; j <= maxIndexY; j++)
         {
            double x = HeightMapTools.toCoordinate(i, heightMap.getGridResolutionXY(), minMaxIndex);
            double y = HeightMapTools.toCoordinate(j, heightMap.getGridResolutionXY(), minMaxIndex);
            double height = heightMap.getHeightAt(i, j);

            if (Double.isNaN(height) || polygonToSnap.distance(new Point2D(x, y)) > 0.01)
            {
               continue;
            }

            pointsInsidePolyon.add(new Point3D(x, y, height));
            averageHeight += height;
         }
      }

      if (pointsInsidePolyon.size() < 3)
      {
         return null;
      }

      planeFitter.fitPlaneToPoints(pointsInsidePolyon, bestFitPlane);
      averageHeight /= pointsInsidePolyon.size();

      double residualSoS = 0.0;
      double totalSoS = 0.0;
      for (int i = 0; i < pointsInsidePolyon.size(); i++)
      {
         double predictedHeight = bestFitPlane.getZOnPlane(pointsInsidePolyon.get(i).getX(), pointsInsidePolyon.get(i).getY());
         residualSoS += MathTools.square(predictedHeight - pointsInsidePolyon.get(i).getZ());
         totalSoS += MathTools.square(averageHeight - pointsInsidePolyon.get(i).getZ());
      }

      rSquared = 1.0 - residualSoS / totalSoS;
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
