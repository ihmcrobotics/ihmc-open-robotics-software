package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.*;

import java.util.List;
import java.util.Random;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

import org.junit.Test;

import us.ihmc.robotics.random.RandomGeometry;

public class IntersectionPlaneBoxCalculatorTest
{
   private static final int NUMBER_OF_ITERATIONS = 10000;
   private static final double EPS = 1.0e-7;

   @Test(timeout = 30000)
   public void testRandomNormals() throws Exception
   {
      Random random = new Random(3424L);

      double cubeSize = 1.0;
      Point3D cubeCenter = new Point3D();
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Point3D pointOnPlane = cubeCenter;
         Vector3D planeNormal = RandomGeometry.nextVector3D(random, 1.0);
         calculator.setCube(cubeSize, cubeCenter);
         calculator.setPlane(pointOnPlane, planeNormal);
         List<Point3D> intersections = calculator.computeIntersections();

         for (int j = 0; j < intersections.size(); j++)
         {
            Point3D intersection = intersections.get(j);
            Vector3D sub = new Vector3D();
            sub.sub(intersection, pointOnPlane);
            assertEquals("Intersection is not on plane", 0.0, sub.dot(planeNormal), EPS);

            Vector3D v0 = new Vector3D();
            Vector3D v1 = new Vector3D();
            Vector3D v3 = new Vector3D();
            Point3D nextIntersection = intersections.get((j + 1) % intersections.size());
            Point3D previousIntersection = intersections.get(j == 0 ? intersections.size() - 1 : j - 1);
            v0.sub(intersection, nextIntersection);
            v1.sub(intersection, previousIntersection);
            v3.cross(v0, v1);
            assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
         }
      }
   }

   @Test(timeout = 30000)
   public void testRandomNormalsAndPointOnPlane() throws Exception
   {
      Random random = new Random(34424L);

      double cubeSize = 1.0;
      Point3D cubeCenter = new Point3D();
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Point3D pointOnPlane = RandomGeometry.nextPoint3D(random, 0.5, 0.5, 0.5);
         Vector3D planeNormal = RandomGeometry.nextVector3D(random, 1.0);
         calculator.setCube(cubeSize, cubeCenter);
         calculator.setPlane(pointOnPlane, planeNormal);
         List<Point3D> intersections = calculator.computeIntersections();

         for (int j = 0; j < intersections.size(); j++)
         {
            Point3D intersection = intersections.get(j);
            Vector3D sub = new Vector3D();
            sub.sub(intersection, pointOnPlane);
            assertEquals("Intersection is not on plane", 0.0, sub.dot(planeNormal), EPS);

            Vector3D v0 = new Vector3D();
            Vector3D v1 = new Vector3D();
            Vector3D v3 = new Vector3D();
            Point3D nextIntersection = intersections.get((j + 1) % intersections.size());
            Point3D previousIntersection = intersections.get(j == 0 ? intersections.size() - 1 : j - 1);
            v0.sub(intersection, nextIntersection);
            v1.sub(intersection, previousIntersection);
            v3.cross(v0, v1);
            assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
         }
      }
   }

   @Test(timeout = 30000)
   public void testRandomNormalsPointOnPlaneAndCubeCenters() throws Exception
   {
      Random random = new Random(3424L);

      double cubeSize = 1.0;
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Point3D cubeCenter = RandomGeometry.nextPoint3D(random, 10.0, 10.0, 10.0);
         Point3D pointOnPlane = RandomGeometry.nextPoint3D(random, 0.5, 0.5, 0.5);
         Vector3D planeNormal = RandomGeometry.nextVector3D(random, 1.0);
         pointOnPlane.add(cubeCenter);
         calculator.setCube(cubeSize, cubeCenter);
         calculator.setPlane(pointOnPlane, planeNormal);
         List<Point3D> intersections = calculator.computeIntersections();

         for (int j = 0; j < intersections.size(); j++)
         {
            Point3D intersection = intersections.get(j);
            Vector3D sub = new Vector3D();
            sub.sub(intersection, pointOnPlane);
            assertEquals("Intersection is not on plane", 0.0, sub.dot(planeNormal), EPS);

            Vector3D v0 = new Vector3D();
            Vector3D v1 = new Vector3D();
            Vector3D v3 = new Vector3D();
            Point3D nextIntersection = intersections.get((j + 1) % intersections.size());
            Point3D previousIntersection = intersections.get(j == 0 ? intersections.size() - 1 : j - 1);
            v0.sub(intersection, nextIntersection);
            v1.sub(intersection, previousIntersection);
            v3.cross(v0, v1);
            if (v3.dot(planeNormal) < 0.0)
            {
               System.err.println("      Point3D cubeCenter = new Point3D" + cubeCenter + ";");
               System.err.println("      Point3D pointOnPlane = new Point3D" + pointOnPlane + ";");
               System.err.println("      Vector3d planeNormal = new Vector3d" + planeNormal + ";");
            }
            assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
         }
      }
   }

   @Test(timeout = 30000)
   public void testBug1() throws Exception
   {
      double cubeSize = 0.1;
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      Point3D cubeCenter = new Point3D(4.25, 0.9500000000000001, 0.75);
      Point3D pointOnPlane = new Point3D(4.200864791870117, 0.9091876149177551, 0.7372332811355591);
      Vector3D planeNormal = new Vector3D(-0.7001400589942932, -0.7001400589942932, 0.14002801477909088);
      calculator.setCube(cubeSize, cubeCenter);
      calculator.setPlane(pointOnPlane, planeNormal);
      List<Point3D> intersections = calculator.computeIntersections();

      for (int j = 0; j < intersections.size(); j++)
      {
         Point3D intersection = intersections.get(j);
         Vector3D sub = new Vector3D();
         sub.sub(intersection, pointOnPlane);
         assertEquals("Intersection is not on plane", 0.0, sub.dot(planeNormal), EPS);

         Vector3D v0 = new Vector3D();
         Vector3D v1 = new Vector3D();
         Vector3D v3 = new Vector3D();
         Point3D nextIntersection = intersections.get((j + 1) % intersections.size());
         Point3D previousIntersection = intersections.get(j == 0 ? intersections.size() - 1 : j - 1);
         v0.sub(intersection, nextIntersection);
         v1.sub(intersection, previousIntersection);
         v3.cross(v0, v1);
         System.out.println(v3.dot(planeNormal) > 0.0);
         assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
      }
   }

   @Test(timeout = 30000)
   public void testBug2() throws Exception
   {
      double cubeSize = 0.1;
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      Point3D cubeCenter = new Point3D(1.35, -1.85, 0.15000000000000002);
      Vector3D planeNormal = new Vector3D(-0.6383859515190125, 0.39992544054985046, 0.6576648950576782);
      Point3D pointOnPlane = new Point3D(1.3115897178649902, -1.8882930278778076, 0.10343723744153976);
      calculator.setCube(cubeSize, cubeCenter);
      calculator.setPlane(pointOnPlane, planeNormal);
      List<Point3D> intersections = calculator.computeIntersections();

      for (int j = 0; j < intersections.size(); j++)
      {
         Point3D intersection = intersections.get(j);
         Vector3D sub = new Vector3D();
         sub.sub(intersection, pointOnPlane);
         assertEquals("Intersection is not on plane", 0.0, sub.dot(planeNormal), EPS);

         Vector3D v0 = new Vector3D();
         Vector3D v1 = new Vector3D();
         Vector3D v3 = new Vector3D();
         Point3D nextIntersection = intersections.get((j + 1) % intersections.size());
         Point3D previousIntersection = intersections.get(j == 0 ? intersections.size() - 1 : j - 1);
         v0.sub(intersection, nextIntersection);
         v1.sub(intersection, previousIntersection);
         v3.cross(v0, v1);
         System.out.println(v3.dot(planeNormal) > 0.0);
         assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
      }
   }

   @Test(timeout = 30000)
   public void testBug3() throws Exception
   {
      double cubeSize = 0.1;
      IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();

      Point3D cubeCenter = new Point3D(-0.25, -0.45, -0.05);
      Point3D pointOnPlane = new Point3D(-0.2242894023656845, -0.4647734463214874, -0.0023258039727807045);
      Vector3D planeNormal = new Vector3D(0.20791170661191224, 1.503689309739766E-8, 0.9781475973766547);
      calculator.setCube(cubeSize, cubeCenter);
      calculator.setPlane(pointOnPlane, planeNormal);
      List<Point3D> intersections = calculator.computeIntersections();

      for (int j = 0; j < intersections.size(); j++)
      {
         Point3D intersection = intersections.get(j);
         Vector3D sub = new Vector3D();
         sub.sub(intersection, pointOnPlane);
         assertEquals("Intersection is not on plane", 0.0, sub.dot(planeNormal), EPS);

         Vector3D v0 = new Vector3D();
         Vector3D v1 = new Vector3D();
         Vector3D v3 = new Vector3D();
         Point3D nextIntersection = intersections.get((j + 1) % intersections.size());
         Point3D previousIntersection = intersections.get(j == 0 ? intersections.size() - 1 : j - 1);
         v0.sub(intersection, nextIntersection);
         v1.sub(intersection, previousIntersection);
         v3.cross(v0, v1);
         System.out.println(v3.dot(planeNormal) > 0.0);
         assertTrue("Intersections are not properly ordered", v3.dot(planeNormal) > 0.0);
      }
   }
}
