package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class LineSegment3dTest
{
   private static Random ran = new Random(100L);

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLenght()
   {
      Point3D point0 = new Point3D(1.0, 1.0, 0.0);
      Point3D point1 = new Point3D(1.0, 1.0, 1.0);
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
      Point3D point0 = new Point3D(0.0, 0.0, 0.0);
      Point3D point1 = new Point3D(1.0, 0.0, 0.0);
      LineSegment3d segment = new LineSegment3d(point0, point1);
      Point3D point = new Point3D(0.0, 0.0, 1.0);
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
      Point3D pointA = new Point3D(1.0, 2.0, 3.0);
      Point3D pointB = new Point3D(1.0, 2.0, 5.0);
      LineSegment3d segmentAB = new LineSegment3d(pointA, pointB);
      Point3D pointC = new Point3D(3.0, 3.0, 1.5); //PointC projection is below pointA
      Point3D expectedPointCprojection = new Point3D(pointA);
      assertEquals(expectedPointCprojection.getX(), segmentAB.orthogonalProjectionCopy(pointC).getX(), 1e-14);
      assertEquals(expectedPointCprojection.getY(), segmentAB.orthogonalProjectionCopy(pointC).getY(), 1e-14);
      assertEquals(expectedPointCprojection.getZ(), segmentAB.orthogonalProjectionCopy(pointC).getZ(), 1e-14);

      pointC = new Point3D(3.0, 3.0, 5.5);
      expectedPointCprojection = new Point3D(pointB); //PointC projection is up pointB	
      assertEquals(expectedPointCprojection.getX(), segmentAB.orthogonalProjectionCopy(pointC).getX(), 1e-14);
      assertEquals(expectedPointCprojection.getY(), segmentAB.orthogonalProjectionCopy(pointC).getY(), 1e-14);
      assertEquals(expectedPointCprojection.getZ(), segmentAB.orthogonalProjectionCopy(pointC).getZ(), 1e-14);

      pointC = new Point3D(3.0, 3.0, 4.5);
      expectedPointCprojection = new Point3D(1.0, 2.0, 4.5); //PointC projection is between pointA and pointB	
      assertEquals(expectedPointCprojection.getX(), segmentAB.orthogonalProjectionCopy(pointC).getX(), 1e-14);
      assertEquals(expectedPointCprojection.getY(), segmentAB.orthogonalProjectionCopy(pointC).getY(), 1e-14);
      assertEquals(expectedPointCprojection.getZ(), segmentAB.orthogonalProjectionCopy(pointC).getZ(), 1e-14);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRandom()
   {
      Point3D pointA = new Point3D(1.0, 2.0, 3.0);
      Point3D pointB = new Point3D(1.0, 2.0, 5.0);
      LineSegment3d segmentAB = new LineSegment3d(pointA, pointB);
      Point3D pointC = new Point3D(0.0, 0.0, 0.0);
      Point3D expectedPointCprojection = new Point3D(0.0, 0.0, 0.0);
      double expectedPointCdistance;

      for (int i = 0; i < 1000; i++)
      {
         pointA.set(randomPoint(500, -250));
         pointB.set(randomPoint(500, -250));
         segmentAB.set(pointA, pointB);
         pointC.set(randomPoint(500, -250));

         Vector3D vectorAB = new Vector3D(pointB);
         vectorAB.sub(pointA);

         Vector3D vectorBA = new Vector3D(vectorAB);
         vectorBA.negate();

         Vector3D vectorAC = new Vector3D(pointC);
         vectorAC.sub(pointA);

         Vector3D vectorBC = new Vector3D(pointC);
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

   private Point3D randomPoint(double absoluteInterval, double offset)
   {
      Point3D returnPoint = new Point3D();
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

      s = new LineSegment3d(new Point3D(0.0, 0.0, 0.0), new Point3D(1.0, 0.0, 0.0));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.0, 0.0, 0.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3D(0.0, 0.0, 0.0), new Point3D(0.0, 1.0, 0.0));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 1.0, 0.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3D(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, 1.0));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, 1.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3D(0.0, 0.0, 0.0), new Point3D(2.0, 0.0, 0.0));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(1.0, 0.0, 0.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3D(0.0, 0.0, 1.0), new Point3D(0.0, 0.0, -3.0));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, -1.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3D(0.0, 0.0, 1.0), new Point3D(0.0, 0.0, -3.0));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(0.0, 0.0, -1.0), s.getDirectionCopy(true), 0.0);

      s = new LineSegment3d(new Point3D(0.0, 0.0, 0.0), new Point3D(Math.cos(Math.PI / 4), Math.sin(Math.PI / 4), 0.0));
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(Math.cos(Math.PI / 4), Math.sin(Math.PI / 4), 0.0), s.getDirectionCopy(true), 0.0);
   }
}
