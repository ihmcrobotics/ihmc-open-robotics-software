package us.ihmc.sensorProcessing.pointClouds.shape;

import georegression.geometry.UtilPlane3D_F64;
import georegression.metric.Distance3D_F64;
import georegression.metric.Intersection3D_F64;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;

import java.util.ArrayList;
import java.util.List;

import org.ddogleg.optimization.FactoryOptimization;
import org.ddogleg.optimization.UnconstrainedLeastSquares;
import org.ddogleg.optimization.UtilOptimize;
import org.ddogleg.optimization.functions.FunctionNtoM;

import us.ihmc.sensorProcessing.pointClouds.shape.ExpectationMaximizationFitter.ScoringFunction;
import bubo.ptcloud.alg.PointVectorNN;

public class CubeCalibration
{
   public static void orient(List<PlaneGeneral3D_F64> planes)
   {
      if (planes.size() < 3)
         return;

      LineParametric3D_F64 line = new LineParametric3D_F64();
      Point3D_F64 point = new Point3D_F64();
      Intersection3D_F64.intersect(planes.get(0), planes.get(1), line);
      Intersection3D_F64.intersect(planes.get(2), line, point);

      System.out.println(point);
   }

   public static void nonLinearFitNormal(final PointVectorNN point)
   {
      // Define the function being optimized and create the optimizer
      final int size = point.neighbors.size();
      final PointVectorNN[] neighbors = point.neighbors.data;
      final Vector3D_F64[] diffs = new Vector3D_F64[neighbors.length];
      for (int i = 0; i < size; i++)
         diffs[i] = new Vector3D_F64(point.p, neighbors[i].p);

      FunctionNtoM func = new FunctionNtoM()
      {
         double TAU_MAX = .0000002; // maximum projection distance
         double LAMBDA = 00.0; // discount rate -> computed as ln(.5)/.05
         double[] n = new double[3];
         double dot;
         Vector3D_F64 a;

         public void process(double[] in, double[] out)
         {
            for (int i = 0; i < size; i++)
            {
               phiThetaToNormal(in[0], in[1], n);

               a = diffs[i];
               dot = (a.x * n[0]) + (a.y * n[1]) + (a.z * n[2]);// - in[2];

               out[i] = Math.pow(Math.E, -LAMBDA * (dot * dot)) * dot;

            }
            //out[point.neighbors.size] = Math.log((TAU_MAX - in[2]) / TAU_MAX);
            //out[point.neighbors.size + 1] = Math.log((TAU_MAX + in[2]) / TAU_MAX);
         }

         public int getN()
         {
            // phi, theta, tau
            return 2;
         }

         public int getM()
         {
            // two log boundary functions
            return neighbors.length + 2;
         }
      };

      UnconstrainedLeastSquares optimizer = FactoryOptimization.leastSquaresLM(1e-3, true);

      // if no jacobian is specified it will be computed numerically
      optimizer.setFunction(func, null);

      // provide it an extremely crude initial estimate of the line equation
      double phi = Math.asin(point.normal.z);
      double theta = Math.asin(point.normal.y / Math.cos(phi));
      optimizer.initialize(new double[] { phi, theta, 0 }, 1e-12, 1e-12);
      //optimizer.initialize(new double[] { 0.2, 0.2}, 1e-12, 1e-12);

      // iterate 500 times or until it converges.
      // Manually iteration is possible too if more control over is required
      UtilOptimize.process(optimizer, 500);

      double found[] = optimizer.getParameters();

      double[] n = new double[3];
      //phiThetaToNormal(phi, theta, n);
      phiThetaToNormal(found[0], found[1], n);
      point.normal = new Vector3D_F64(n[0], n[1], n[2]);
   }

   public static void weightedLinearFit(final PointVectorNN point, ScoringFunction<PlaneNormal3D_F64, Point3D_F64> scorer, int iterations)
   {
      // Define the function being optimized and create the optimizer
      final int size = point.neighbors.size();
      final PointVectorNN[] neighbors = point.neighbors.data;
      
      List<Point3D_F64> diffs = new ArrayList<Point3D_F64>();
      for (int i = 0; i < size; i++) {
         Vector3D_F64 v = new Vector3D_F64(point.p, neighbors[i].p);
         diffs.add(i, new Point3D_F64(v.x, v.y, v.z));
      }
         
      FitPlaneWeighted3D_F64 fitter = new FitPlaneWeighted3D_F64();
      Point3D_F64 fitOrigin = new Point3D_F64();
      Vector3D_F64 fitNormal = new Vector3D_F64();
      
      PlaneNormal3D_F64 plane = new PlaneNormal3D_F64(point.p, point.normal);

      double[] weights = new double[size];

      for (; iterations > 0; iterations--)
      {
         getWeights(weights, plane, diffs, scorer);

         for (int i = 0; i < weights.length; i++)
         {
            fitter.svd(diffs, weights, fitOrigin, fitNormal);
            plane = new PlaneNormal3D_F64(fitOrigin, fitNormal);
         }
      }

      point.normal = fitNormal;
   }

   public static double[] getWeights(double[] weights, PlaneNormal3D_F64 plane, List<Point3D_F64> allPoints, ScoringFunction<PlaneNormal3D_F64, Point3D_F64> scorer)
   {
      if (weights == null)
         weights = new double[allPoints.size()];

      for (int i = 0; i < allPoints.size(); i++)
      {
         weights[i] = scorer.score(plane, allPoints.get(i));
      }

      return weights;
   }

   public static void phiThetaToNormal(double phi, double theta, double[] n)
   {
      n[0] = Math.cos(phi) * Math.cos(theta);
      n[1] = Math.cos(phi) * Math.sin(theta);
      n[2] = Math.sin(phi);
   }
}
