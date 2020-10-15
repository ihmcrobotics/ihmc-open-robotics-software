package us.ihmc.robotics.statistics;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLine2D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class Line2DStandardDeviationCalculatorTest
{
   private static final double epsilon = 1e-6;

   @Test
   public void testNoVariance()
   {
      YoRegistry testRegistry = new YoRegistry(getClass().getSimpleName());
      YoFrameLine2D valueProvider = new YoFrameLine2D("valueProvider", ReferenceFrame.getWorldFrame(), testRegistry);
      Line2DStatisticsCalculator calculator = new Line2DStatisticsCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      Vector2D direction = new Vector2D(17.3, 5.6);
      Point2D position = new Point2D(17.3, 5.6);
      direction.normalize();
      valueProvider.getDirection().set(direction);
      valueProvider.getPoint().set(position);

      for (int i = 0; i < numberOfValues; i++)
      {
         calculator.update();
      }

      EuclidGeometryTestTools.assertLine2DGeometricallyEquals(valueProvider, calculator.getLineMean(), epsilon);
      assertEquals(0.0, calculator.getDirectionStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getPositionStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getDirectionVariance(), epsilon);
      assertEquals(0.0, calculator.getPositionVariance(), epsilon);
   }

   @Test
   public void testNoVarianceWithOppositeDirections()
   {
      YoRegistry testRegistry = new YoRegistry(getClass().getSimpleName());
      YoFrameLine2D valueProvider = new YoFrameLine2D("valueProvider", ReferenceFrame.getWorldFrame(), testRegistry);
      Line2DStatisticsCalculator calculator = new Line2DStatisticsCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      Vector2D direction = new Vector2D(17.3, 5.6);
      Point2D position = new Point2D(17.3, 5.6);
      direction.normalize();
      valueProvider.getDirection().set(direction);
      valueProvider.getPoint().set(position);

      for (int i = 0; i < numberOfValues; i++)
      {
         direction.negate();
         valueProvider.getDirection().set(direction);
         calculator.update();
      }

      EuclidGeometryTestTools.assertLine2DGeometricallyEquals(valueProvider, calculator.getLineMean(), epsilon);
      assertEquals(0.0, calculator.getDirectionStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getPositionStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getDirectionVariance(), epsilon);
      assertEquals(0.0, calculator.getPositionVariance(), epsilon);
   }

   @Test
   public void testNoVarianceAlongLine()
   {
      YoRegistry testRegistry = new YoRegistry(getClass().getSimpleName());
      YoFrameLine2D valueProvider = new YoFrameLine2D("valueProvider", ReferenceFrame.getWorldFrame(), testRegistry);
      Line2DStatisticsCalculator calculator = new Line2DStatisticsCalculator("value", valueProvider, testRegistry);

      int numberOfValues = 100;
      Vector2D direction = new Vector2D(17.3, 5.6);
      Point2D position = new Point2D(17.3, 5.6);
      direction.normalize();
      valueProvider.getDirection().set(direction);
      valueProvider.getPoint().set(position);

      Line2D originalValue = new Line2D(valueProvider);

      Random random = new Random(1738L);

      for (int i = 0; i < numberOfValues; i++)
      {
         Point2D modifiedPosition = new Point2D(position);
         Vector2D positionModification = new Vector2D(direction);
         positionModification.scale(RandomNumbers.nextDouble(random, 10.0));
         modifiedPosition.add(positionModification);

         valueProvider.getPoint().set(modifiedPosition);

         calculator.update();
      }

      EuclidGeometryTestTools.assertLine2DGeometricallyEquals(originalValue, calculator.getLineMean(), epsilon);
      assertEquals(0.0, calculator.getPositionVariance(), epsilon);
      assertEquals(0.0, calculator.getDirectionVariance(), epsilon);
      assertEquals(0.0, calculator.getDirectionStandardDeviation(), epsilon);
      assertEquals(0.0, calculator.getPositionStandardDeviation(), epsilon);
   }
}
