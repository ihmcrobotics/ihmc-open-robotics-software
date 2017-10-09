package us.ihmc.sensorProcessing.pointClouds.shape;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.apache.commons.math3.distribution.NormalDistribution;

import georegression.geometry.UtilPlane3D_F64;
import georegression.metric.Distance3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;

public class ExpectationMaximizationFitter
{
   public static List<PlaneGeneral3D_F64> fit(int numPlanes, List<Point3D_F64> allPoints, ScoringFunction<PlaneGeneral3D_F64, Point3D_F64> scorer, int iterations)
   {
      return fit(numPlanes, new Random(), allPoints, scorer, iterations);
   }

   public static List<PlaneGeneral3D_F64> fit(int numPlanes, Random rand, List<Point3D_F64> allPoints, ScoringFunction<PlaneGeneral3D_F64, Point3D_F64> scorer, int iterations)
   {
      List<PlaneGeneral3D_F64> planes = new ArrayList<PlaneGeneral3D_F64>();
      for (int i = 0; i < numPlanes; i++)
      {
         planes.add(new PlaneGeneral3D_F64(rand.nextDouble(), rand.nextDouble(), rand.nextDouble(), rand.nextDouble()));
      }
      System.out.println(planes.size());
      return fit(planes, allPoints, scorer, iterations);
   }

   public static List<PlaneGeneral3D_F64> fit(List<PlaneGeneral3D_F64> planes, List<Point3D_F64> allPoints, ScoringFunction<PlaneGeneral3D_F64, Point3D_F64> scorer, int iterations)
   {
      FitPlaneWeighted3D_F64 fitter = new FitPlaneWeighted3D_F64();
      Point3D_F64 fitOrigin = new Point3D_F64();
      Vector3D_F64 fitNormal = new Vector3D_F64();

      double[][] weights = new double[planes.size()][allPoints.size()];

      for (; iterations > 0; iterations--)
      {
         getWeights(weights, planes, allPoints, scorer);

         planes.clear();
         for (int i = 0; i < weights.length; i++)
         {
            fitter.svd(allPoints, weights[i], fitOrigin, fitNormal);
            planes.add(UtilPlane3D_F64.convert(new PlaneNormal3D_F64(fitOrigin, fitNormal), null));

         }
      }

      return planes;
   }

   public static double[][] getWeights(double[][] weights, List<PlaneGeneral3D_F64> planes, List<Point3D_F64> allPoints, ScoringFunction<PlaneGeneral3D_F64, Point3D_F64> scorer)
   {
      if(weights == null)
         weights = new double[planes.size()][allPoints.size()];
      
      double[] probabilities = new double[planes.size()];
      double sumProbability;

      for (int i = 0; i < allPoints.size(); i++)
      {

         sumProbability = 0;
         for (int j = 0; j < planes.size(); j++)
         {
            probabilities[j] = scorer.score(planes.get(j), allPoints.get(i));
            sumProbability += probabilities[j];
         }

         for (int j = 0; j < probabilities.length; j++)
         {
            if (sumProbability != 0)
            {
               probabilities[j] /= sumProbability;
               weights[j][i] = probabilities[j];
            }
            else
            {
               weights[j][i] = 0;
            }
         }
      }
      
      return weights;
   }

   public static interface ScoringFunction <T,E>
   {
      public double score(T model, E sample);
   }

   public static ScoringFunction<PlaneGeneral3D_F64, Point3D_F64> getGaussianSqauresMixedError(double sigma)
   {
      final NormalDistribution d = new NormalDistribution(0, sigma);
      final double distAmplitude = d.density(0);

      return new ScoringFunction<PlaneGeneral3D_F64, Point3D_F64>()
      {
         public double score(PlaneGeneral3D_F64 plane, Point3D_F64 point)
         {
            double e = Distance3D_F64.distance(plane, point);
            // gaussian best for fitting, small 1/(e*e) component for large errors 
            return d.density(e) + distAmplitude * 1e-12 / (1e-10 + (e * e));
         }
      };
   }
   
   public static ScoringFunction<PlaneNormal3D_F64, Point3D_F64> getNormalSingularityError()
   {
      return new ScoringFunction<PlaneNormal3D_F64, Point3D_F64>()
      {
         public double score(PlaneNormal3D_F64 plane, Point3D_F64 diff)
         {
            double e = plane.n.x * diff.x + plane.n.y * diff.y + plane.n.z * diff.z;
            e /= diff.norm();
            return 1 / (1e-2 + (e));
         }
      };
   }
}
