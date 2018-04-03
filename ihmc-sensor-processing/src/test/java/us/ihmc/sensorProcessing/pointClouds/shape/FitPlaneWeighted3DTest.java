package us.ihmc.sensorProcessing.pointClouds.shape;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import georegression.fitting.plane.FitPlane3D_F64;
import georegression.geometry.UtilPlane3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class FitPlaneWeighted3DTest
{
   Random rand = new Random(1231L);
   double eps = 3e-7;

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testAgainstUnweighted() {
      for (int i = 0; i<100; i++) {
         int n = rand.nextInt(500) + 4;
         testAgainstUnweighted(n, rand.nextInt(n-3)+3);
      }
   }
   
   public void testAgainstUnweighted(int n, int k)
   {
      Point3D_F64 origin = new Point3D_F64();
      Vector3D_F64 normal = new Vector3D_F64();

      List<Point3D_F64> allPoints = getRandomPoints(n);
      List<Point3D_F64> selectedPoints = new ArrayList<Point3D_F64>();
      double[] weights = new double[n];
      
      while (selectedPoints.size() < k) {
         int index = rand.nextInt(allPoints.size());
         Point3D_F64 point = allPoints.get(index);
         if (!selectedPoints.contains(point)) {
            selectedPoints.add(point);
            weights[index] = 1;
         }
      }

      new FitPlaneWeighted3D_F64().svd(allPoints, weights, origin, normal);
      PlaneGeneral3D_F64 weightedFit = UtilPlane3D_F64.convert(new PlaneNormal3D_F64(origin, normal), null);
      new FitPlane3D_F64().svd(selectedPoints, origin, normal);
      PlaneGeneral3D_F64 fit = UtilPlane3D_F64.convert(new PlaneNormal3D_F64(origin, normal), null);

      assert(epsilonEquals(weightedFit, fit, eps));
   }

   public boolean epsilonEquals(PlaneGeneral3D_F64 a, PlaneGeneral3D_F64 b, double eps)
   {
      double magA = Math.sqrt(a.A * a.A + a.B * a.B + a.C * a.C + a.D * a.D);
      double magB = Math.sqrt(b.A * b.A + b.B * b.B + b.C * b.C + b.D * b.D);

      return Math.abs(a.A / magA - b.A / magB) < eps && 
            Math.abs(a.B / magA - b.B / magB) < eps && 
            Math.abs(a.C / magA - b.C / magB) < eps && 
            Math.abs(a.D / magA - b.D / magB) < eps;
   }

   public List<Point3D_F64> getRandomPoints(int n)
   {
      List<Point3D_F64> points = new ArrayList<Point3D_F64>();
      for (int i = 0; i < n; i++)
      {
         points.add(new Point3D_F64(rand.nextDouble(), rand.nextDouble(), rand.nextDouble()));
      }
      return points;
   }
}
