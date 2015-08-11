package us.ihmc.robotics.geometry;

import org.junit.Ignore;
import org.junit.Test;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.robotics.geometry.shapes.Plane3d;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.test.JUnitTools;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertTrue;

public class LeastSquaresZPlaneFitterTest
{
	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleFlatCase()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3d> pointList = new ArrayList<Point3d>();
      Plane3d plane3d = new Plane3d();

      pointList.add(new Point3d(0.0, 0.0, 0.0));
      pointList.add(new Point3d(0.0, 0.1, 0.0));
      pointList.add(new Point3d(1.0, 0.1, 0.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);

      JUnitTools.assertTuple3dEquals(new Vector3d(0.0, 0.0, 1.0), plane3d.getNormalCopy(), 1e-7);
   }

	@EstimatedDuration(duration = 0.0)
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
         Point3d planePoint = RandomTools.generateRandomPoint(random, maxXYZ, maxXYZ, maxXYZ);
         Vector3d planeNormal = RandomTools.generateRandomVector(random, 1.0);

         if (planeNormal.getZ() < 0.0)
            planeNormal.scale(-1.0);

         Plane3d plane3d = new Plane3d(planePoint, planeNormal);

         int numberOfPoints = RandomTools.generateRandomInt(random, 3, 50);
         ArrayList<Point3d> listOfPoints = new ArrayList<Point3d>();

         for (int j = 0; j < numberOfPoints; j++)
         {
            Point3d point = RandomTools.generateRandomPoint(random, maxXYZ, maxXYZ, maxXYZ);
            plane3d.orthogonalProjection(point);

            point.add(RandomTools.generateRandomVector(random, pointNoiseAmplitude));
            
            listOfPoints.add(point);
         }

         Plane3d plane3dSolution = new Plane3d();
         leastSquaresZPlaneFitter.fitPlaneToPoints(listOfPoints, plane3dSolution);

         Point3d pointSolution = plane3dSolution.getPointCopy();
         Vector3d normalSolution = plane3dSolution.getNormalCopy();

         JUnitTools.assertTuple3dEquals(planeNormal, normalSolution, normalEpsilon);

         double distanceToPlane = plane3d.distance(pointSolution);
         assertTrue(distanceToPlane < pointEpsilon);
      }
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testCornerCaseWithOnlyTwoPoints()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3d> pointList = new ArrayList<Point3d>();
      Plane3d plane3d = new Plane3d();

      pointList.add(new Point3d(0.0, 0.0, 0.0));
      pointList.add(new Point3d(0.0, 0.1, 0.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);

      assertTrue(isNaN(plane3d.getPointCopy()));
      assertTrue(isNaN(plane3d.getNormalCopy()));
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testCornerCaseWithColinearPoints()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3d> pointList = new ArrayList<Point3d>();
      Plane3d plane3d = new Plane3d();

      pointList.add(new Point3d(0.0, 0.0, 0.0));
      pointList.add(new Point3d(0.0, 0.1, 0.0));
      pointList.add(new Point3d(0.0, 0.3, 0.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);

      assertTrue(isNaN(plane3d.getPointCopy()));
      assertTrue(isNaN(plane3d.getNormalCopy()));
   }
   
   
   private boolean isNaN(Tuple3d tuple3d)
   {
      if (!Double.isNaN(tuple3d.getX())) return false;
      if (!Double.isNaN(tuple3d.getY())) return false;
      if (!Double.isNaN(tuple3d.getZ())) return false;
      
      return true;
   }

   @Ignore("Straight up and down fails with LeastSquaresZPlaneFitter since it assumes equation Ax + By + z + C = 0")

	@EstimatedDuration
	@Test(timeout=300000)
   public void testStraightUpAndDownPlane()
   {
      LeastSquaresZPlaneFitter leastSquaresZPlaneFitter = new LeastSquaresZPlaneFitter();

      List<Point3d> pointList = new ArrayList<Point3d>();
      Plane3d plane3d = new Plane3d();

      pointList.add(new Point3d(0.0, 0.0, 0.0));
      pointList.add(new Point3d(0.0, 0.1, 0.0));
      pointList.add(new Point3d(0.0, 0.05, 1.0));

      leastSquaresZPlaneFitter.fitPlaneToPoints(pointList, plane3d);

      JUnitTools.assertTuple3dEquals(new Vector3d(0.0, 1.0, 0.0), plane3d.getNormalCopy(), 1e-7);
   }

  
   

}
