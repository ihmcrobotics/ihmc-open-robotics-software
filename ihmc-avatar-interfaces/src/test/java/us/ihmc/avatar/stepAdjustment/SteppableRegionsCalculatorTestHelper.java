package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;
import java.util.Random;

public class SteppableRegionsCalculatorTestHelper
{
   private static final int numberOfPointsToCheck = 1000;

   public static void assertSteppableRegionsAreValid(Random random, List<StepConstraintRegion> steppableRegions, PlanarRegionsList allPlanarRegions)
   {
      for (StepConstraintRegion steppableRegion : steppableRegions)
      {
         // check all around the perimeter
         for (int vertexIdx = 0; vertexIdx < steppableRegion.getConcaveHullSize(); vertexIdx++)
         {
            Point2DReadOnly vertex = steppableRegion.getConcaveHull().getVertex(vertexIdx);
            Point2DReadOnly nextVertex = steppableRegion.getConcaveHull().getNextVertex(vertexIdx);

            for (int i = 0; i < 100; i++)
            {
               Point2D pointInRegion = new Point2D();
               pointInRegion.interpolate(vertex, nextVertex, i / 100);

               Point3D pointInWorld = new Point3D(pointInRegion);
               steppableRegion.getTransformToWorld().transform(pointInWorld);

               Point3D projectedPointInWorld = PlanarRegionTools.projectPointToPlanesVertically(pointInWorld, allPlanarRegions);

               EuclidCoreTestTools.assertPoint3DGeometricallyEquals(pointInWorld, projectedPointInWorld, 1e-7);
            }
         }

         int iter = 0;
         while (iter < numberOfPointsToCheck)
         {
            Point2DReadOnly interiorPoint = getRandomPointInside(random, steppableRegion.getConvexHullInConstraintRegion());
            boolean isValidInteriorPoint = steppableRegion.isPointInside(interiorPoint.getX(), interiorPoint.getY());
            for (int i = 0; i < steppableRegion.getNumberOfHolesInRegion(); i++)
            {
               if (steppableRegion.getHoleInConstraintRegion(i).isPointInside(interiorPoint.getX(), interiorPoint.getY()))
               {
                  isValidInteriorPoint = false;
                  break;
               }
            }

            if (isValidInteriorPoint)
            {
               Point3D pointInWorld = new Point3D(interiorPoint);
               steppableRegion.getTransformToWorld().transform(pointInWorld);

               Point3D projectedPointInWorld = PlanarRegionTools.projectPointToPlanesVertically(pointInWorld, allPlanarRegions);

               EuclidCoreTestTools.assertPoint3DGeometricallyEquals(pointInWorld, projectedPointInWorld, 1e-7);

               iter++;
            }
         }

      }
   }

   private static Point2DReadOnly getRandomPointInside(Random random, ConvexPolygon2DReadOnly polygon)
   {
      double maxValue = 1.0;
      Point2DBasics pointToReturn = new Point2D();
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         double alpha = RandomNumbers.nextDouble(random, 0.0, maxValue);
         maxValue -= alpha;

         pointToReturn.scaleAdd(alpha, polygon.getVertex(i), pointToReturn);
      }

      return pointToReturn;
   }

}
