package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.geometry.shapes.Plane3d;
import us.ihmc.robotics.random.RandomTools;

public class LeastSquaresZPlaneFitterTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPointsWithSamePitchAndDifferentPositionGetSameAnswer()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3D> pointListA = new ArrayList<Point3D>();
      Plane3d plane3dA = new Plane3d();

      pointListA.add(new Point3D( 1.0,  1.0,  0.1));
      pointListA.add(new Point3D( 1.0, -1.0,  0.1));
      pointListA.add(new Point3D(-1.0,  1.0, -0.1));
      pointListA.add(new Point3D(-1.0, -1.0, -0.1));
      
      leastSquaresZPlaneFitter.fitPlaneToPoints(pointListA, plane3dA);
      Vector3D normalA = plane3dA.getNormalCopy();
      
      List<Point3D> pointListB = new ArrayList<Point3D>();
      Plane3d plane3dB = new Plane3d();
      pointListB.add(new Point3D( 1.0 + 4.0,  1.0 + 4.0,  0.1));
      pointListB.add(new Point3D( 1.0 + 4.0, -1.0 + 4.0,  0.1));
      pointListB.add(new Point3D(-1.0 + 4.0,  1.0 + 4.0, -0.1));
      pointListB.add(new Point3D(-1.0 + 4.0, -1.0 + 4.0, -0.1));
      
      leastSquaresZPlaneFitter.fitPlaneToPoints(pointListB, plane3dB);
      Vector3D normalB = plane3dB.getNormalCopy();
      
      assertTrue(normalA.epsilonEquals(normalB, 1e-7));
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleFlatCase()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3D> pointList = new ArrayList<Point3D>();
      Plane3d plane3d = new Plane3d();

      pointList.add(new Point3D(0.0, 0.0, 0.0));
      pointList.add(new Point3D(0.0, 0.1, 0.0));
      pointList.add(new Point3D(1.0, 0.1, 0.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);

      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 0.0, 1.0), plane3d.getNormalCopy(), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
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
         Point3D planePoint = RandomTools.generateRandomPoint(random, maxXYZ, maxXYZ, maxXYZ);
         Vector3D planeNormal = RandomTools.generateRandomVector(random, 1.0);

         if (planeNormal.getZ() < 0.0)
            planeNormal.scale(-1.0);

         Plane3d plane3d = new Plane3d(planePoint, planeNormal);

         int numberOfPoints = RandomTools.generateRandomInt(random, 3, 50);
         ArrayList<Point3D> listOfPoints = new ArrayList<Point3D>();

         for (int j = 0; j < numberOfPoints; j++)
         {
            Point3D point = RandomTools.generateRandomPoint(random, maxXYZ, maxXYZ, maxXYZ);
            plane3d.orthogonalProjection(point);

            point.add(RandomTools.generateRandomVector(random, pointNoiseAmplitude));
            
            listOfPoints.add(point);
         }

         Plane3d plane3dSolution = new Plane3d();
         leastSquaresZPlaneFitter.fitPlaneToPoints(listOfPoints, plane3dSolution);

         Point3D pointSolution = plane3dSolution.getPointCopy();
         Vector3D normalSolution = plane3dSolution.getNormalCopy();

         EuclidCoreTestTools.assertTuple3DEquals(planeNormal, normalSolution, normalEpsilon);

         double distanceToPlane = plane3d.distance(pointSolution);
         assertTrue(distanceToPlane < pointEpsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCornerCaseWithOnlyTwoPoints()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3D> pointList = new ArrayList<Point3D>();
      Plane3d plane3d = new Plane3d();

      pointList.add(new Point3D(0.0, 0.0, 0.0));
      pointList.add(new Point3D(0.0, 0.1, 0.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);

      assertTrue(isNaN(plane3d.getPointCopy()));
      assertTrue(isNaN(plane3d.getNormalCopy()));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCornerCaseWithColinearPoints()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3D> pointList = new ArrayList<Point3D>();
      Plane3d plane3d = new Plane3d();

      pointList.add(new Point3D(0.0, 0.0, 0.0));
      pointList.add(new Point3D(0.0, 0.1, 0.0));
      pointList.add(new Point3D(0.0, 0.3, 0.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);

      assertTrue(isNaN(plane3d.getPointCopy()));
      assertTrue(isNaN(plane3d.getNormalCopy()));
   }
   
   
   private boolean isNaN(Tuple3DBasics tuple3d)
   {
      if (!Double.isNaN(tuple3d.getX())) return false;
      if (!Double.isNaN(tuple3d.getY())) return false;
      if (!Double.isNaN(tuple3d.getZ())) return false;
      
      return true;
   }

   // Straight up and down fails with LeastSquaresZPlaneFitter since it assumes equation Ax + By + z + C = 0
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testStraightUpAndDownPlane()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3D> pointList = new ArrayList<Point3D>();
      Plane3d plane3d = new Plane3d();

      pointList.add(new Point3D(0.0, 0.0, 0.0));
      pointList.add(new Point3D(0.0, 0.1, 0.0));
      pointList.add(new Point3D(0.0, 0.05, 1.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);

      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 1.0, 0.0), plane3d.getNormalCopy(), 1e-7);
   }

  
   

}
