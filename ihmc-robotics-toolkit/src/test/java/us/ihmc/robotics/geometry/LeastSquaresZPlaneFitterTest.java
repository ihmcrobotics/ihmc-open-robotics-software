package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

import static org.junit.jupiter.api.Assertions.*;

public class LeastSquaresZPlaneFitterTest
{
   @Test
   public void testPointsWithSamePitchAndDifferentPositionGetSameAnswer()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3D> pointListA = new ArrayList<Point3D>();
      Plane3D plane3dA = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

      pointListA.add(new Point3D( 1.0,  1.0,  0.1));
      pointListA.add(new Point3D( 1.0, -1.0,  0.1));
      pointListA.add(new Point3D(-1.0,  1.0, -0.1));
      pointListA.add(new Point3D(-1.0, -1.0, -0.1));
      
      leastSquaresZPlaneFitter.fitPlaneToPoints(pointListA, plane3dA);
      Vector3D normalA = new Vector3D(plane3dA.getNormal());
      
      List<Point3D> pointListB = new ArrayList<Point3D>();
      Plane3D plane3dB = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      pointListB.add(new Point3D( 1.0 + 4.0,  1.0 + 4.0,  0.1));
      pointListB.add(new Point3D( 1.0 + 4.0, -1.0 + 4.0,  0.1));
      pointListB.add(new Point3D(-1.0 + 4.0,  1.0 + 4.0, -0.1));
      pointListB.add(new Point3D(-1.0 + 4.0, -1.0 + 4.0, -0.1));
      
      leastSquaresZPlaneFitter.fitPlaneToPoints(pointListB, plane3dB);
      Vector3D normalB = new Vector3D(plane3dB.getNormal());

      EuclidCoreTestTools.assertEquals(normalA, normalB, 1e-7);
   }
   
	@Test
   public void testSimpleFlatCase()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3D> pointList = new ArrayList<Point3D>();
      Plane3D plane3d = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

      pointList.add(new Point3D(0.0, 0.0, 0.0));
      pointList.add(new Point3D(0.0, 0.1, 0.0));
      pointList.add(new Point3D(1.0, 0.1, 0.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);

      EuclidCoreTestTools.assertEquals(new Vector3D(0.0, 0.0, 1.0), new Vector3D(plane3d.getNormal()), 1e-7);
   }

	@Test
   public void testRandomlyGeneratedPointsOnRandomPlanes()
   {
      int numberOfTests = 1000;
      double maxXYZ = 10.0;

      Random random = new Random(1678L);

      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();
      double pointNoiseAmplitude = 0.0;
      double normalEpsilon = 1e-10;
      double pointEpsilon = 1e-10;
      
      performATestWithRandomPoints(numberOfTests, maxXYZ, random, leastSquaresZPlaneFitter, pointNoiseAmplitude, normalEpsilon, pointEpsilon);
      
      pointNoiseAmplitude = 0.001;
      normalEpsilon = 1e-2;
      pointEpsilon = 1e-3;
      
      performATestWithRandomPoints(numberOfTests, maxXYZ, random, leastSquaresZPlaneFitter, pointNoiseAmplitude, normalEpsilon, pointEpsilon);
      
      pointNoiseAmplitude = 0.01;
      normalEpsilon = 0.03;
      pointEpsilon = 0.01;
      
      performATestWithRandomPoints(numberOfTests, maxXYZ, random, leastSquaresZPlaneFitter, pointNoiseAmplitude, normalEpsilon, pointEpsilon);
   }

   private static void performATestWithRandomPoints(int numberOfTests,
                                                    double maxXYZ,
                                                    Random random,
                                                    LeastSquaresZPlaneFitter leastSquaresZPlaneFitter,
                                                    double pointNoiseAmplitude,
                                                    double normalEpsilon,
                                                    double pointEpsilon)
   {
      for (int i = 0; i < numberOfTests; i++)
      {
         Point3D planePoint = EuclidCoreRandomTools.nextPoint3D(random, maxXYZ, maxXYZ, maxXYZ);
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         if (planeNormal.getZ() < 0.0)
            planeNormal.scale(-1.0);

         Plane3D plane3d = new Plane3D(planePoint, planeNormal);

         int numberOfPoints = RandomNumbers.nextInt(random, 3, 50);
         ArrayList<Point3D> listOfPoints = new ArrayList<Point3D>();

         for (int j = 0; j < numberOfPoints; j++)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, maxXYZ, maxXYZ, maxXYZ);
            plane3d.orthogonalProjection(point);

            point.add(EuclidCoreRandomTools.nextVector3D(random, pointNoiseAmplitude));
            
            listOfPoints.add(point);
         }

         Plane3D plane3dSolution = new Plane3D();
         leastSquaresZPlaneFitter.fitPlaneToPoints(listOfPoints, plane3dSolution);

         EuclidCoreTestTools.assertEquals(planeNormal, plane3dSolution.getNormal(), normalEpsilon);

         double distanceToPlane = plane3d.distance(plane3dSolution.getPoint());
         assertTrue(distanceToPlane < pointEpsilon);
      }
   }

	@Test
   public void testCornerCaseWithOnlyTwoPoints()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3D> pointList = new ArrayList<Point3D>();
      Plane3D plane3d = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

      pointList.add(new Point3D(0.0, 0.0, 0.0));
      pointList.add(new Point3D(0.0, 0.1, 0.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);
      assertTrue(plane3d.containsNaN());
   }

	@Test
   public void testCornerCaseWithColinearPoints()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3D> pointList = new ArrayList<Point3D>();
      Plane3D plane3d = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

      pointList.add(new Point3D(0.0, 0.0, 0.0));
      pointList.add(new Point3D(0.0, 0.1, 0.0));
      pointList.add(new Point3D(0.0, 0.3, 0.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);
      assertTrue(plane3d.containsNaN());
   }

   @Test
   public void testSquaredError()
   {
      Random random = new Random(342);
      int numTests = 30;

      for (int i = 0; i < numTests; i++)
      {
         List<Point3D> points = new ArrayList<>();

         int numPoints = RandomNumbers.nextInt(random, 5, 50);
         for (int j = 0; j < numPoints; j++)
         {
            double px = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double py = EuclidCoreRandomTools.nextDouble(random, 10.0);
            double pz = EuclidCoreRandomTools.nextDouble(random, 1.0);
            points.add(new Point3D(px, py, pz));
         }

         LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();
         Plane3D plane = new Plane3D();
         double computedSquaredError = planeFitter.fitPlaneToPoints(points, plane);

         // compute actual squared error
         double error = 0.0;
         for (int j = 0; j < points.size(); j++)
         {
            error += EuclidCoreTools.square(points.get(j).getZ() - plane.getZOnPlane(points.get(j).getX(), points.get(j).getY()));
         }
         error /= points.size();

         Assertions.assertTrue(Math.abs(computedSquaredError - error) < 1.0e-15);
      }
   }

   // Straight up and down fails with LeastSquaresZPlaneFitter since it assumes equation Ax + By + z + C = 0
	@Disabled
	@Test
   public void testStraightUpAndDownPlane()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3D> pointList = new ArrayList<Point3D>();
      Plane3D plane3d = new Plane3D(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

      pointList.add(new Point3D(0.0, 0.0, 0.0));
      pointList.add(new Point3D(0.0, 0.1, 0.0));
      pointList.add(new Point3D(0.0, 0.05, 1.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);

      EuclidCoreTestTools.assertEquals(new Vector3D(0.0, 1.0, 0.0), new Vector3D(plane3d.getNormal()), 1e-7);
   }
}
