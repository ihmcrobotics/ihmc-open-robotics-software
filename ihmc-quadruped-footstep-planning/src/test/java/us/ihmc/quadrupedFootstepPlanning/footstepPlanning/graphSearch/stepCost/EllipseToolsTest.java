package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class EllipseToolsTest
{
   @Test
   public void computeMagnitudeOnEllipseInDirection()
   {
      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         double maxX = RandomNumbers.nextDouble(random, 0.0, 100.0);
         double maxY = RandomNumbers.nextDouble(random, 0.0, 100.0);

         double xDirection = RandomNumbers.nextDouble(random, -10.0, 10.0);
         double yDirection = RandomNumbers.nextDouble(random, -10.0, 10.0);

         double xIntersection = maxX * maxY / Math.sqrt(MathTools.square(maxX * yDirection) + MathTools.square(maxY * xDirection)) * xDirection;
         double yIntersection = maxX * maxY / Math.sqrt(MathTools.square(maxX * yDirection) + MathTools.square(maxY * xDirection)) * yDirection;

         double magnitude = EuclidCoreTools.norm(xIntersection, yIntersection);

         assertEquals(magnitude, EllipseTools.computeMagnitudeOnEllipseInDirection(maxX, maxY, xDirection, yDirection), 1e-5);
      }
   }

   @Test
   public void testProjectPointOntoEllipse()
   {
      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         double maxX = RandomNumbers.nextDouble(random, 0.0, 100.0);
         double maxY = RandomNumbers.nextDouble(random, 0.0, 100.0);

         Point2DReadOnly pointToProject = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         double projectedXPosition = maxX * maxY / Math.sqrt(MathTools.square(maxX * pointToProject.getY()) + MathTools.square(maxY * pointToProject.getX())) * pointToProject.getX();
         double projectedYPosition = maxX * maxY / Math.sqrt(MathTools.square(maxX * pointToProject.getY()) + MathTools.square(maxY * pointToProject.getX())) * pointToProject.getY();

         Point2D projectedPointExpected = new Point2D(projectedXPosition, projectedYPosition);

         Point2D projectedPoint = new Point2D();
         EllipseTools.projectPointOntoEllipse(maxX, maxY, pointToProject, projectedPoint);
         EuclidCoreTestTools.assertPoint2DGeometricallyEquals(projectedPointExpected, projectedPoint, 1e-5);
      }
   }

   @Test
   public void testDistanceToEllipse()
   {
      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         double maxX = RandomNumbers.nextDouble(random, 0.0, 100.0);
         double maxY = RandomNumbers.nextDouble(random, 0.0, 100.0);

         Point2DReadOnly point = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         double projectedXPosition = maxX * maxY / Math.sqrt(MathTools.square(maxX * point.getY()) + MathTools.square(maxY * point.getX())) * point.getX();
         double projectedYPosition = maxX * maxY / Math.sqrt(MathTools.square(maxX * point.getY()) + MathTools.square(maxY * point.getX())) * point.getY();

         Point2D projectedPoint = new Point2D(projectedXPosition, projectedYPosition);

         double distanceSign = EllipseTools.isPointInsideEllipse(maxX, maxY, point) ? -1.0 : 1.0;
         double expectedDistance = distanceSign * projectedPoint.distance(point);
         double distance = EllipseTools.getDistanceToEllipse(maxX, maxY, point);

         assertEquals("iteration " + i, expectedDistance, distance, 1e-6);
      }
   }

   @Test
   public void testIsPointInsideEllipse()
   {
      double epsilonToEdge = 1e-2;
      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         double maxX = RandomNumbers.nextDouble(random, 0.0, 100.0);
         double maxY = RandomNumbers.nextDouble(random, 0.0, 100.0);

         Point2DReadOnly point = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         double distanceToEllipse = EllipseTools.getDistanceToEllipse(maxX, maxY, point);
         boolean pointIsInsideExpected = distanceToEllipse < 0.0;

         assertEquals("iteration " + i, pointIsInsideExpected, EllipseTools.isPointInsideEllipse(maxX, maxY, point));
      }
   }

}
