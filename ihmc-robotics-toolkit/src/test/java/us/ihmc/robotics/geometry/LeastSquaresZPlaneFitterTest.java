package us.ihmc.robotics.geometry;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.random.RandomGeometry;

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
      
      assertTrue(normalA.epsilonEquals(normalB, 1e-7));
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

      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 0.0, 1.0), new Vector3D(plane3d.getNormal()), 1e-7);
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

   
   private void performATestWithRandomPoints(int numberOfTests, double maxXYZ, Random random, LeastSquaresZPlaneFitter leastSquaresZPlaneFitter,
         double pointNoiseAmplitude, double normalEpsilon, double pointEpsilon)
   {
      for (int i = 0; i < numberOfTests; i++)
      {
         Point3D planePoint = RandomGeometry.nextPoint3D(random, maxXYZ, maxXYZ, maxXYZ);
         Vector3D planeNormal = RandomGeometry.nextVector3D(random, 1.0);

         if (planeNormal.getZ() < 0.0)
            planeNormal.scale(-1.0);

         Plane3D plane3d = new Plane3D(planePoint, planeNormal);

         int numberOfPoints = RandomNumbers.nextInt(random, 3, 50);
         ArrayList<Point3D> listOfPoints = new ArrayList<Point3D>();

         for (int j = 0; j < numberOfPoints; j++)
         {
            Point3D point = RandomGeometry.nextPoint3D(random, maxXYZ, maxXYZ, maxXYZ);
            plane3d.orthogonalProjection(point);

            point.add(RandomGeometry.nextVector3D(random, pointNoiseAmplitude));
            
            listOfPoints.add(point);
         }

         Plane3D plane3dSolution = new Plane3D();
         leastSquaresZPlaneFitter.fitPlaneToPoints(listOfPoints, plane3dSolution);

         Point3D pointSolution = new Point3D(plane3dSolution.getPoint());
         Vector3D normalSolution = new Vector3D(plane3dSolution.getNormal());

         EuclidCoreTestTools.assertTuple3DEquals(planeNormal, normalSolution, normalEpsilon);

         double distanceToPlane = plane3d.distance(pointSolution);
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

      assertTrue(isNaN(new Point3D(plane3d.getPoint())));
      assertTrue(isNaN(new Vector3D(plane3d.getNormal())));
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

      assertTrue(isNaN(new Point3D(plane3d.getPoint())));
      assertTrue(isNaN(new Vector3D(plane3d.getNormal())));
   }
   
   
   private boolean isNaN(Tuple3DBasics tuple3d)
   {
      if (!Double.isNaN(tuple3d.getX())) return false;
      if (!Double.isNaN(tuple3d.getY())) return false;
      if (!Double.isNaN(tuple3d.getZ())) return false;
      
      return true;
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

      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 1.0, 0.0), new Vector3D(plane3d.getNormal()), 1e-7);
   }

  
   

}
