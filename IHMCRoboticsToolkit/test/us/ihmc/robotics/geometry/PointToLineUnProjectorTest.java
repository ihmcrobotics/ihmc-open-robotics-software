package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.random.RandomTools;

public class PointToLineUnProjectorTest
{
   private static final double eps = 1e-7;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testsimpleCase()
   {
      double x0 = 1.0;
      double y0 = 0.0;
      double z0 = 0.0;
      double x1 = 0.0;
      double y1 = 0.0;
      double z1 = 0.0;
      double x2 = 0.0;
      double y2 = 0.0;
      double z2 = 0.0;
      runTest(x0, y0, z0, x1, y1, z1, x2, y2, z2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGeneralCase()
   {
      Random gen = new Random(124L);
      for (int i = 0; i < 100000; i++)
      {
         Point3D point1 = RandomTools.generateRandomPoint(gen, -1000, -1000, -1000, 1000, 1000, 1000);
         Point3D point2 = RandomTools.generateRandomPoint(gen, -1000, -1000, -1000, 1000, 1000, 1000);
         double s = RandomNumbers.nextDouble(gen, 0, 1);
         Point3D temp = new Point3D(point1);
         temp.scale(1 - s);
         Point3D point3 = new Point3D(temp);
         temp.set(point2);
         temp.scale(s);
         point3.add(temp);
         runTest(point1.getX(), point1.getY(), point1.getZ(), point2.getX(), point2.getY(), point2.getZ(), point3.getX(), point3.getY(), point3.getZ());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testDegenerateCase()
   {
      Random gen = new Random(124L);
      for (int i = 0; i < 100000; i++)
      {
         Point3D point1 = RandomTools.generateRandomPoint(gen, -1000, -1000, -1000, 1000, 1000, 1000);
         Point3D point2 = new Point3D(point1);
         double s = RandomNumbers.nextDouble(gen, 0, 1);
         Point3D temp = new Point3D(point1);
         temp.scale(1 - s);
         Point3D point3 = new Point3D(temp);
         temp.set(point2);
         temp.scale(s);
         point3.add(temp);
         runTest(point1.getX(), point1.getY(), point1.getZ(), point2.getX(), point2.getY(), point2.getZ(), point3.getX(), point3.getY(), point3.getZ());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testYCase()
   {
      Random gen = new Random(124L);
      for (int i = 0; i < 100000; i++)
      {
         Point3D point1 = RandomTools.generateRandomPoint(gen, -1000, -1000, -1000, 1000, 1000, 1000);
         Point3D point2 = RandomTools.generateRandomPoint(gen, -1000, -1000, -1000, 1000, 1000, 1000);
         point2.setY(point1.getY());
         double s = RandomNumbers.nextDouble(gen, 0, 1);
         Point3D temp = new Point3D(point1);
         temp.scale(1 - s);
         Point3D point3 = new Point3D(temp);
         temp.set(point2);
         temp.scale(s);
         point3.add(temp);
         runTest(point1.getX(), point1.getY(), point1.getZ(), point2.getX(), point2.getY(), point2.getZ(), point3.getX(), point3.getY(), point3.getZ());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testXCase()
   {
      Random gen = new Random(124L);
      for (int i = 0; i < 100000; i++)
      {
         Point3D point1 = RandomTools.generateRandomPoint(gen, -1000, -1000, -1000, 1000, 1000, 1000);
         Point3D point2 = RandomTools.generateRandomPoint(gen, -1000, -1000, -1000, 1000, 1000, 1000);
         point2.setX(point1.getX());
         double s = RandomNumbers.nextDouble(gen, 0, 1);
         Point3D temp = new Point3D(point1);
         temp.scale(1 - s);
         Point3D point3 = new Point3D(temp);
         temp.set(point2);
         temp.scale(s);
         point3.add(temp);
         runTest(point1.getX(), point1.getY(), point1.getZ(), point2.getX(), point2.getY(), point2.getZ(), point3.getX(), point3.getY(), point3.getZ());
      }
   }

   private void runTest(double x0, double y0, double z0, double x1, double y1, double z1, double x2, double y2, double z2)
   {
      PointToLineUnProjector unProjector = new PointToLineUnProjector();

      unProjector.setLine(new Point2D(x0, y0), new Point2D(x1, y1), z0, z1);
      assertEquals(z2, unProjector.unProject(x2, y2), eps);
   }
}
