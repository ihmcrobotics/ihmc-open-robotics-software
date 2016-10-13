package us.ihmc.robotics.geometry;

import us.ihmc.robotics.geometry.LineSegment1d;
import us.ihmc.tools.testing.MutationTestingTools;

import org.junit.Test;

import static org.junit.Assert.*;

public class LineSegment1dTest
{
   @Test
   public void lineBoundariesTest()
   {
      double firstPoint = 16.7;
      double secondPoint = -2.5;
      double p1 = 16.7;
      double p2 = -2.5;
      double p3 = 0.0;
      double p4 = 7.1;
      double p5 = -5.2;

      LineSegment1d line = new LineSegment1d(firstPoint, secondPoint);

      assertEquals(line.getMinPoint(), p2, 0.001);
      assertEquals(line.getMaxPoint(), p1, 0.001);
      assertEquals(line.getSecondEndpoint(), p2, 0.001);
      assertEquals(line.getFirstEndpoint(), p1, 0.001);
      assertEquals(line.getMidPoint(), p4, 0.001);

      assertTrue(line.isBetweenEndpoints(p3, 0.001));
      assertFalse(line.isBetweenEndpoints(p5, 0.001));
      assertFalse(line.isBetweenEndpointsExclusive(p1));
      assertTrue(line.isBetweenEndpointsInclusive(p2));

      LineSegment1d pointLine = new LineSegment1d(firstPoint, firstPoint);

      assertTrue(pointLine.isBetweenEndpoints(p1, 0.0));
      assertFalse(pointLine.isBetweenEndpoints(p1, 0.01));
   }

   @Test
   public void lineOverlapsTest()
   {
      double firstPoint = -10;
      double secondPoint = 10;

      LineSegment1d emptyLine = new LineSegment1d();
      LineSegment1d mainLine = new LineSegment1d(firstPoint, secondPoint);
      LineSegment1d separateLine = new LineSegment1d(firstPoint + 100, secondPoint + 100);
      LineSegment1d otherLine1 = new LineSegment1d(firstPoint - 3, secondPoint - 3);
      LineSegment1d intersectionLine1 = new LineSegment1d(firstPoint, secondPoint - 3);
      LineSegment1d otherLine2 = new LineSegment1d(secondPoint, secondPoint + 10);

      assertTrue(mainLine.computeOverlap(otherLine1, emptyLine));
      assertTrue(intersectionLine1.computeOverlap(emptyLine, emptyLine));
      assertFalse(mainLine.computeOverlap(separateLine, emptyLine));

      assertTrue(mainLine.isOverlappingInclusive(otherLine2));
      assertFalse(mainLine.isOverlappingExclusive(otherLine2));
   }

   @Test
   public void testDistance()
   {
      double firstPoint = -10;
      double secondPoint = 10;
      double p1 = 15;
      double p2 = -15;
      double p3 = 0.0;

      LineSegment1d mainLine = new LineSegment1d(firstPoint, secondPoint);

      assertEquals(mainLine.distance(p1), 5, 0.001);
      assertEquals(mainLine.distance(p2), 5, 0.001);
      assertEquals(mainLine.distance(p3), -10, 0.001);
      
      assertTrue(mainLine.isBefore(p2));
      assertTrue(mainLine.isAfter(p1));
      assertFalse(mainLine.isBefore(p3));
      assertFalse(mainLine.isAfter(p3));

      assertEquals(mainLine.length(), 20, 0.001);
   }
   
   @Test
   public void testExtension()
   {
      double firstPoint = -10;
      double secondPoint = 10;
      double p1 = 15;
      double p2 = -15;
      double p3 = 0.0;

      LineSegment1d mainLine = new LineSegment1d(firstPoint, secondPoint);
      
      mainLine.extendSegmentToPoint(p1);
      assertEquals(mainLine.getMaxPoint(), p1, 0.001);
      mainLine.extendSegmentToPoint(p2);
      assertEquals(mainLine.getMinPoint(), p2, 0.001);
      mainLine.extendSegmentToPoint(p3);
      assertEquals(mainLine.getMaxPoint(), p1, 0.001);
      assertEquals(mainLine.getMinPoint(), p2, 0.001);
   }
   
   public static void main(String[] args)
   {
      String targetTests = "us.ihmc.robotics.geometry.LineSegment1dTest";
      String targetClasses = "us.ihmc.robotics.geometry.LineSegment1d";
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }

}
