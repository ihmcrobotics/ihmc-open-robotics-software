package us.ihmc.robotics.functionApproximation;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class OnlineLine2DLinearRegressionTest
{
   private static final int iters = 100;
   private static final double epsilon = 1e-8;

   @Test
   public void testLineFinding()
   {
      Random random = new Random(1738L);

      int pointsInEstimate = 10;
      OnlineLine2DLinearRegression linearRegression = new OnlineLine2DLinearRegression("", new YoRegistry("test"));

      for (int i = 0; i < iters; i++)
      {
         Line2DReadOnly lineActual = EuclidGeometryRandomTools.nextLine2D(random);
         linearRegression.reset();


         List<Point2DReadOnly> pointsOnLine = new ArrayList<>();

         for (int pointIdx = 0; pointIdx < pointsInEstimate; pointIdx++)
         {
            double distanceOnLine = RandomNumbers.nextDouble(random, 10.0);
            Point2D pointOnLine = new Point2D();
            pointOnLine.scaleAdd(distanceOnLine, lineActual.getDirection(), lineActual.getPoint());
            pointsOnLine.add(pointOnLine);
            linearRegression.update(pointOnLine);

            assertTrue(lineActual.isPointOnLine(pointOnLine));
         }

         assertTrue(lineActual.isPointOnLine(linearRegression.getMeanLine().getPoint()));
         Point2DReadOnly meanPoint = computeMeanPoint(pointsOnLine);

         EuclidCoreTestTools.assertPoint2DGeometricallyEquals(meanPoint, linearRegression.getMeanLine().getPoint(), epsilon);
         assertEquals(computeXStandardDeviation(meanPoint, pointsOnLine), linearRegression.getXStandardDeviation(), epsilon);
         assertEquals(computeYStandardDeviation(meanPoint, pointsOnLine), linearRegression.getYStandardDeviation(), epsilon);

         if (!lineActual.getDirection().epsilonEquals(linearRegression.getMeanLine().getDirection(), epsilon))
         {
            // check other direction
            Vector2D direction = new Vector2D(linearRegression.getMeanLine().getDirection());
            direction.negate();
            EuclidCoreTestTools.assertVector2DGeometricallyEquals(lineActual.getDirection(), direction, epsilon);
         }

         assertEquals("Iter " + i + " failed.", 0.0, linearRegression.getTransverseStandardDeviation(), epsilon);
         assertEquals("Iter " + i + " failed.", computeInlineStandardDeviation(meanPoint, pointsOnLine), linearRegression.getInlineStandardDeviation(), epsilon);


      }
   }

   @Test
   public void testNoisyLineFinding()
   {
      Random random = new Random(1738L);

      int pointsInEstimate = 10000;
      OnlineLine2DLinearRegression linearRegression = new OnlineLine2DLinearRegression("", new YoRegistry("test"));

      for (int i = 0; i < 10; i++)
      {
         Line2DReadOnly lineActual = EuclidGeometryRandomTools.nextLine2D(random);
         linearRegression.reset();

         double transverseDeviation = RandomNumbers.nextDouble(random, 0.0, 0.05);
         double inlineDeviation = RandomNumbers.nextDouble(random, 0.0, 0.6);

         List<Point2DReadOnly> pointsOnLine = new ArrayList<>();

         for (int pointIdx = 0; pointIdx < pointsInEstimate; pointIdx++)
         {
            double transverseTranslation = transverseDeviation * random.nextGaussian();
            double inlineTranslation = inlineDeviation * random.nextGaussian();
            Vector2D transverseOffset = EuclidGeometryTools.perpendicularVector2D(lineActual.getDirection());
            transverseOffset.scale(transverseTranslation / transverseOffset.length());
            Vector2D inlineOffset = new Vector2D(lineActual.getDirection());
            inlineOffset.scale(inlineTranslation / inlineOffset.length());

            Point2D pointOnLine = new Point2D(lineActual.getPoint());
            pointOnLine.add(inlineOffset);
            pointOnLine.add(transverseOffset);

            linearRegression.update(pointOnLine);

            pointsOnLine.add(pointOnLine);
         }

//         assertTrue(lineActual.isPointOnLine(linearRegression.getMeanLine().getPoint()));
         Point2DReadOnly meanPoint = computeMeanPoint(pointsOnLine);

         EuclidCoreTestTools.assertPoint2DGeometricallyEquals(meanPoint, linearRegression.getMeanLine().getPoint(), epsilon);
         assertEquals(computeXStandardDeviation(meanPoint, pointsOnLine), linearRegression.getXStandardDeviation(), epsilon);
         assertEquals(computeYStandardDeviation(meanPoint, pointsOnLine), linearRegression.getYStandardDeviation(), epsilon);

         double angle = lineActual.getDirection().angle(linearRegression.getMeanLine().getDirection());
         assertEquals("Iter " + i + " failed.", transverseDeviation, linearRegression.getTransverseStandardDeviation(), 0.04);
         assertEquals("Iter " + i + " failed.", inlineDeviation, linearRegression.getInlineStandardDeviation(), 0.03);


      }
   }

   private static Point2DReadOnly computeMeanPoint(List<Point2DReadOnly> points)
   {
      Point2D meanPoint = new Point2D();
      for (Point2DReadOnly point : points)
         meanPoint.add(point);

      meanPoint.scale(1.0 / points.size());

      return meanPoint;
   }

   private static double computeXStandardDeviation(Point2DReadOnly meanPoint, List<Point2DReadOnly> points)
   {
      double variance = 0.0;
      for (Point2DReadOnly point : points)
         variance += MathTools.square(point.getX() - meanPoint.getX());
      variance /= points.size();

      return Math.sqrt(variance);
   }

   private static double computeYStandardDeviation(Point2DReadOnly meanPoint, List<Point2DReadOnly> points)
   {
      double variance = 0.0;
      for (Point2DReadOnly point : points)
         variance += MathTools.square(point.getY() - meanPoint.getY());
      variance /= points.size();

      return Math.sqrt(variance);
   }

   private static double computeInlineStandardDeviation(Point2DReadOnly meanPoint, List<Point2DReadOnly> points)
   {
      double variance = 0.0;
      for (Point2DReadOnly point : points)
         variance += point.distanceSquared(meanPoint);
      variance /= points.size();

      return Math.sqrt(variance);
   }

}
