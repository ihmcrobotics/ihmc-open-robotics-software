package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

public class ConvexPolygon2dTestHelpers
{

   public static ArrayList<FrameConvexPolygon2D> generateRandomPolygons(Random random, ReferenceFrame zUpFrame, double centerXMin, double centerXMax, double centerYMin,
         double centerYMax, double widthMax, double heightMax, int numberOfPoints, int numberOfPolygons)
   {
      ArrayList<FrameConvexPolygon2D> ret = new ArrayList<FrameConvexPolygon2D>(numberOfPolygons);
      for (int i = 0; i < numberOfPolygons; i++)
      {
         FramePoint2D center = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, centerXMin, centerXMax, centerYMin, centerYMax);

         //       double x2A = xMin + widthMax/2.0 + random.nextDouble() * (xMax - xMin - widthMax);
         //       double x2B = xMin + widthMax/2.0 + random.nextDouble() * (xMax - xMin - widthMax);
         //
         //       double y2A = yMin + heightMax/2.0 + random.nextDouble() * (yMax - yMin - heightMax);
         //       double y2B = yMin + heightMax/2.0 + random.nextDouble() * (yMax - yMin - heightMax);
         //
         double xMin2 = center.getX() - widthMax / 2.0;
         double xMax2 = center.getX() + widthMax / 2.0;

         double yMin2 = center.getY() - heightMax / 2.0;
         double yMax2 = center.getY() + heightMax / 2.0;

         FrameConvexPolygon2D polygon = generateRandomPolygon(random, zUpFrame, xMin2, xMax2, yMin2, yMax2, numberOfPoints);

         ret.add(polygon);
      }

      return ret;
   }

   public static FrameConvexPolygon2D generateRandomPolygon(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin, double yMax,
         int numberOfPoints)
   {
      FramePoint2D randomExtents1 = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, xMin, xMax, yMin, yMax);
      FramePoint2D randomExtents2 = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, xMin, xMax, yMin, yMax);

      double xMin2 = Math.min(randomExtents1.getX(), randomExtents2.getX());
      double xMax2 = Math.max(randomExtents1.getX(), randomExtents2.getX());

      double yMin2 = Math.min(randomExtents1.getY(), randomExtents2.getY());
      double yMax2 = Math.max(randomExtents1.getY(), randomExtents2.getY());

      ArrayList<FramePoint2D> points = generateRandomCircularFramePoints(random, zUpFrame, xMin2, xMax2, yMin2, yMax2, numberOfPoints);

      return new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(points));
   }

   public static ArrayList<FramePoint2D> generateRandomCircularFramePoints(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin,
         double yMax, int numberOfPoints)
   {
      ArrayList<FramePoint2D> points = new ArrayList<FramePoint2D>();

      FramePoint2D zeroFramePoint = new FramePoint2D(zUpFrame, (xMax + xMin) / 2.0, (yMax + yMin) / 2.0);

      for (int i = 0; i < numberOfPoints; i++)
      {
         FramePoint2D randomPoint = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, xMin, xMax, yMin, yMax);

         if (randomPoint.distance(zeroFramePoint) > (Math.max((xMax - xMin) / 2.0, (yMax - yMin) / 2.0)))
            continue;

         points.add(randomPoint);
      }

      return points;
   }

   public static ArrayList<FramePoint2D> generateRandomRectangularFramePoints(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin,
         double yMax, int numberOfPoints)
   {
      ArrayList<FramePoint2D> points = new ArrayList<FramePoint2D>();

      for (int i = 0; i < numberOfPoints; i++)
      {
         FramePoint2D randomPoint = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, xMin, xMax, yMin, yMax);

         points.add(randomPoint);
      }

      return points;
   }


   public static void verifyPointsAreInside(FrameConvexPolygon2D polygon, ArrayList<FramePoint2D> pointsThatShouldBeInside, double epsilon)
   {
      for (FramePoint2D point : pointsThatShouldBeInside)
      {
         if (!polygon.isPointInside(point, epsilon))
         {
            throw new RuntimeException("Point is not inside polygon. Point = " + point);
         }
      }
   }

   public static void verifyPointsAreNotInside(FrameConvexPolygon2D polygon, ArrayList<FramePoint2D> pointsThatShouldNotBeInside, double epsilon)
   {
      for (FramePoint2D point : pointsThatShouldNotBeInside)
      {
         if (polygon.isPointInside(point, epsilon))
         {
            throw new RuntimeException("Point is inside polygon. Point = " + point);
         }
      }
   }

}
