package us.ihmc.sensorProcessing.pointClouds.shape;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import georegression.fitting.plane.FitPlane3D_F64;
import georegression.geometry.UtilPlane3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;

public class FitPlaneWeighted3DTest
{
   Random rand = new Random(1231L);
   double eps = 3e-7;

   @Test
   public void testAgainstUnweighted()
   {
      for (int i = 0; i < 100; i++)
      {
         int n = rand.nextInt(500) + 4;
         testAgainstUnweighted(i, n, rand.nextInt(n - 3) + 3);
      }
   }

   public void testAgainstUnweighted(int iteration, int n, int k)
   {
      Point3D_F64 origin = new Point3D_F64();
      Vector3D_F64 normal = new Vector3D_F64();

      List<Point3D_F64> allPoints = getRandomPoints(n);
      List<Point3D_F64> selectedPoints = new ArrayList<>();
      double[] weights = new double[n];

      while (selectedPoints.size() < k)
      {
         int index = rand.nextInt(allPoints.size());
         Point3D_F64 point = allPoints.get(index);
         if (!selectedPoints.contains(point))
         {
            selectedPoints.add(point);
            weights[index] = 1;
         }
      }

      new FitPlaneWeighted3D_F64().svd(allPoints, weights, origin, normal);
      PlaneGeneral3D_F64 weightedFit = UtilPlane3D_F64.convert(new PlaneNormal3D_F64(origin, normal), null);
      new FitPlane3D_F64().svd(selectedPoints, origin, normal);
      PlaneGeneral3D_F64 fit = UtilPlane3D_F64.convert(new PlaneNormal3D_F64(origin, normal), null);

      assertTrue(epsilonEquals(weightedFit, fit, eps), "Iteration: " + iteration);
   }

   public boolean epsilonEquals(PlaneGeneral3D_F64 a, PlaneGeneral3D_F64 b, double eps)
   {
      double magA = Math.sqrt(a.A * a.A + a.B * a.B + a.C * a.C + a.D * a.D);
      double magB = Math.sqrt(b.A * b.A + b.B * b.B + b.C * b.C + b.D * b.D);

      if (Math.abs(a.A / magA - b.A / magB) >= eps)
         return false;
      if (Math.abs(a.B / magA - b.B / magB) >= eps)
         return false;
      if (Math.abs(a.C / magA - b.C / magB) >= eps)
         return false;
      if (Math.abs(a.D / magA - b.D / magB) >= eps)
         return false;

      return true;
   }

   public List<Point3D_F64> getRandomPoints(int n)
   {
      List<Point3D_F64> points = new ArrayList<>();
      for (int i = 0; i < n; i++)
      {
         points.add(new Point3D_F64(rand.nextDouble(), rand.nextDouble(), rand.nextDouble()));
      }
      return points;
   }
}
