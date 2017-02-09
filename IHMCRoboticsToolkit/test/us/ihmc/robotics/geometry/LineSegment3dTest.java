package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

public class LineSegment3dTest
{
   private static Random ran = new Random(100L);

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLenght()
   {
      Point3d point0 = new Point3d(1.0, 1.0, 0.0);
      Point3d point1 = new Point3d(1.0, 1.0, 1.0);
      LineSegment3d segment = new LineSegment3d(point0, point1);
      assertEquals(1.0, segment.length(), 1e-14);

      segment.setFirstEndpoint(Math.sqrt(3), Math.sqrt(3), Math.sqrt(3));
      segment.setSecondEndpoint(Math.sqrt(3), Math.sqrt(3), Math.sqrt(3));
      assertEquals(0.0, segment.length(), 1e-14);

      segment.setSecondEndpoint(-Math.sqrt(3), -Math.sqrt(3), -Math.sqrt(3));
      assertEquals(6.0, segment.length(), 1e-14);

      segment.setSecondEndpoint(0.0, 0.0, 0.0);
      assertEquals(3.0, segment.length(), 1e-14);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceToPoint()
   {
      Point3d point0 = new Point3d(0.0, 0.0, 0.0);
      Point3d point1 = new Point3d(1.0, 0.0, 0.0);
      LineSegment3d segment = new LineSegment3d(point0, point1);
      Point3d point = new Point3d(0.0, 0.0, 1.0);
      assertEquals(1.0, segment.distance(point), 1e-14);

      point.set(0.5, 0.0, 1.0);
      assertEquals(1.0, segment.distance(point), 1e-14);

      point.set(1.0, 0.0, 1.0);
      assertEquals(1.0, segment.distance(point), 1e-14);

      point.set(-1.0, 0.0, 1.0);
      assertEquals(Math.sqrt(2.0), segment.distance(point), 1e-14);

      point.set(2.0, 0.0, 1.0);
      assertEquals(Math.sqrt(2.0), segment.distance(point), 1e-14);

      segment.set(0.0, 0.0, 0.0, 2.0, 2.0, 0.0);
      point.set(2.0, 0.0, 0.0);
      assertEquals(2.0 / Math.sqrt(2.0), segment.distance(point), 1e-14);

      point.set(3.0, 0.0, 0.0);
      assertEquals(3.0 / Math.sqrt(2.0), segment.distance(point), 1e-14);

      point.set(4.0, 0.0, 0.0);
      assertEquals(4.0 / Math.sqrt(2.0), segment.distance(point), 1e-14);

      segment.setSecondEndpoint(-2.0, -2.0, -2.0);
      point.set(2.0, 2.0, 2.0);
      assertEquals(segment.length(), segment.distance(point), 1e-14);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testProjection()
   {
      Point3d pointA = new Point3d(1.0, 2.0, 3.0);
      Point3d pointB = new Point3d(1.0, 2.0, 5.0);
      LineSegment3d segmentAB = new LineSegment3d(pointA, pointB);
      Point3d pointC = new Point3d(3.0, 3.0, 1.5); //PointC projection is below pointA
      Point3d expectedPointCprojection = new Point3d(pointA);
      assertEquals(expectedPointCprojection.getX(), segmentAB.orthogonalProjectionCopy(pointC).getX(), 1e-14);
      assertEquals(expectedPointCprojection.getY(), segmentAB.orthogonalProjectionCopy(pointC).getY(), 1e-14);
      assertEquals(expectedPointCprojection.getZ(), segmentAB.orthogonalProjectionCopy(pointC).getZ(), 1e-14);

      pointC = new Point3d(3.0, 3.0, 5.5);
      expectedPointCprojection = new Point3d(pointB); //PointC projection is up pointB	
      assertEquals(expectedPointCprojection.getX(), segmentAB.orthogonalProjectionCopy(pointC).getX(), 1e-14);
      assertEquals(expectedPointCprojection.getY(), segmentAB.orthogonalProjectionCopy(pointC).getY(), 1e-14);
      assertEquals(expectedPointCprojection.getZ(), segmentAB.orthogonalProjectionCopy(pointC).getZ(), 1e-14);

      pointC = new Point3d(3.0, 3.0, 4.5);
      expectedPointCprojection = new Point3d(1.0, 2.0, 4.5); //PointC projection is between pointA and pointB	
      assertEquals(expectedPointCprojection.getX(), segmentAB.orthogonalProjectionCopy(pointC).getX(), 1e-14);
      assertEquals(expectedPointCprojection.getY(), segmentAB.orthogonalProjectionCopy(pointC).getY(), 1e-14);
      assertEquals(expectedPointCprojection.getZ(), segmentAB.orthogonalProjectionCopy(pointC).getZ(), 1e-14);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRandom()
   {
      Point3d pointA = new Point3d(1.0, 2.0, 3.0);
      Point3d pointB = new Point3d(1.0, 2.0, 5.0);
      LineSegment3d segmentAB = new LineSegment3d(pointA, pointB);
      Point3d pointC = new Point3d(0.0, 0.0, 0.0);
      Point3d expectedPointCprojection = new Point3d(0.0, 0.0, 0.0);
      double expectedPointCdistance;

      for (int i = 0; i < 1000; i++)
      {
         pointA.set(randomPoint(500, -250));
         pointB.set(randomPoint(500, -250));
         segmentAB.set(pointA, pointB);
         pointC.set(randomPoint(500, -250));

         Vector3d vectorAB = new Vector3d(pointB);
         vectorAB.sub(pointA);

         Vector3d vectorBA = new Vector3d(vectorAB);
         vectorBA.negate();

         Vector3d vectorAC = new Vector3d(pointC);
         vectorAC.sub(pointA);

         Vector3d vectorBC = new Vector3d(pointC);
         vectorBC.sub(pointB);

         if (vectorAB.dot(vectorAC) <= 0)
         {
            expectedPointCprojection.set(pointA);

         }
         else
         {
            if (vectorBA.dot(vectorBC) <= 0)
            {
               expectedPointCprojection.set(pointB);
            }
            else
            {
               double ratio = vectorAB.dot(vectorAC) / segmentAB.length();
               vectorAB.normalize();
               expectedPointCprojection.set(vectorAB);
               expectedPointCprojection.scale(ratio);
               expectedPointCprojection.add(pointA);
            }
         }

         expectedPointCdistance = pointC.distance(expectedPointCprojection);

         assertEquals(expectedPointCprojection.getX(), segmentAB.orthogonalProjectionCopy(pointC).getX(), 1e-12);
         assertEquals(expectedPointCprojection.getY(), segmentAB.orthogonalProjectionCopy(pointC).getY(), 1e-12);
         assertEquals(expectedPointCprojection.getZ(), segmentAB.orthogonalProjectionCopy(pointC).getZ(), 1e-12);

         assertEquals(expectedPointCdistance, segmentAB.distance(pointC), 1e-12);
      }
   }

   private Point3d randomPoint(double absoluteInterval, double offset)
   {
      Point3d returnPoint = new Point3d();
      returnPoint.setX(ran.nextDouble() * absoluteInterval + offset);
      returnPoint.setY(ran.nextDouble() * absoluteInterval + offset);
      returnPoint.setZ(ran.nextDouble() * absoluteInterval + offset);
      return returnPoint;
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void directionTest()
   {
      LineSegment3d s;

      s = new LineSegment3d(new Point3d(0.0, 0.0, 0.0), new Point3d(1.0, 0.0, 0.0));
      JUnitTools.assertTuple3dEquals(new Point3d(1.0, 0.0, 0.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3d(0.0, 0.0, 0.0), new Point3d(0.0, 1.0, 0.0));
      JUnitTools.assertTuple3dEquals(new Point3d(0.0, 1.0, 0.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3d(0.0, 0.0, 0.0), new Point3d(0.0, 0.0, 1.0));
      JUnitTools.assertTuple3dEquals(new Point3d(0.0, 0.0, 1.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3d(0.0, 0.0, 0.0), new Point3d(2.0, 0.0, 0.0));
      JUnitTools.assertTuple3dEquals(new Point3d(1.0, 0.0, 0.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3d(0.0, 0.0, 1.0), new Point3d(0.0, 0.0, -3.0));
      JUnitTools.assertTuple3dEquals(new Point3d(0.0, 0.0, -1.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3d(0.0, 0.0, 1.0), new Point3d(0.0, 0.0, -3.0));
      JUnitTools.assertTuple3dEquals(new Point3d(0.0, 0.0, -1.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3d(0.0, 0.0, 0.0), new Point3d(Math.cos(Math.PI / 4), Math.sin(Math.PI / 4), 0.0));
      JUnitTools.assertTuple3dEquals(new Point3d(Math.cos(Math.PI / 4), Math.sin(Math.PI / 4), 0.0), s.getDirectionCopy(true), 0.0);
   }
}
