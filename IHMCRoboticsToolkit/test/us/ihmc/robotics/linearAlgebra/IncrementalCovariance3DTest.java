package us.ihmc.robotics.linearAlgebra;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.junit.Test;

import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.random.RandomTools;

public class IncrementalCovariance3DTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testEasyCase()
   {
      Random random = new Random(51651L);
      IncrementalCovariance3D incrementalCovariance3D = new IncrementalCovariance3D();

      Point3d average = new Point3d();
      int length = 100;
      Vector3d maxAmplitude = new Vector3d(1.0, 1.0, 1.0);

      for (int i = 0; i < 100; i++)
      {
         List<Point3d> dataset = createRandomDataset(random, average, length, maxAmplitude);
         
         incrementalCovariance3D.clear();
         incrementalCovariance3D.addAllDataPoints(dataset);
         assertCovarianceIsCorrect(incrementalCovariance3D, dataset);
         
         incrementalCovariance3D.clear();
         for (Point3d dataPoint : dataset)
            incrementalCovariance3D.addDataPoint(dataPoint);
         assertCovarianceIsCorrect(incrementalCovariance3D, dataset);
         
         int index = random.nextInt(length);
         incrementalCovariance3D.removeDataPoint(dataset.get(index));
         dataset.remove(index);
         assertCovarianceIsCorrect(incrementalCovariance3D, dataset);
      }
   }

   @Test
   public void testNonZeroMean()
   {
      Random random = new Random(51651L);
      IncrementalCovariance3D incrementalCovariance3D = new IncrementalCovariance3D();

      int length = 100;
      Vector3d maxAmplitude = new Vector3d(1.0, 1.0, 1.0);

      for (int i = 0; i < 100; i++)
      {
         Point3d average = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
         List<Point3d> dataset = createRandomDataset(random, average, length, maxAmplitude);

         incrementalCovariance3D.clear();
         incrementalCovariance3D.addAllDataPoints(dataset);
         assertCovarianceIsCorrect(incrementalCovariance3D, dataset);
         
         incrementalCovariance3D.clear();
         for (Point3d dataPoint : dataset)
            incrementalCovariance3D.addDataPoint(dataPoint);
         assertCovarianceIsCorrect(incrementalCovariance3D, dataset);
         
         int index = random.nextInt(length);
         incrementalCovariance3D.removeDataPoint(dataset.get(index));
         dataset.remove(index);
         assertCovarianceIsCorrect(incrementalCovariance3D, dataset);

         incrementalCovariance3D.clearAndSetPredictedMean(average);
         incrementalCovariance3D.addAllDataPoints(dataset);
         assertCovarianceIsCorrect(incrementalCovariance3D, dataset);
      }
   }

   private void assertCovarianceIsCorrect(IncrementalCovariance3D incrementalCovariance3D, List<Point3d> dataset)
   {
      DenseMatrix64F expectedCovariance;
      DenseMatrix64F actualCovariance = new DenseMatrix64F(0, 0);
      incrementalCovariance3D.getCovariance(actualCovariance);
      expectedCovariance = computeCovarianceMatrix(dataset, false);
      assertTrue(MatrixFeatures.isEquals(actualCovariance, expectedCovariance, EPSILON));

      incrementalCovariance3D.getCovarianceCorrected(actualCovariance);
      expectedCovariance = computeCovarianceMatrix(dataset, true);
      assertTrue(MatrixFeatures.isEquals(actualCovariance, expectedCovariance, EPSILON));
   }

   private static List<Point3d> createRandomDataset(Random random, Point3d average, int length, Vector3d maxAmplitude)
   {
      List<Point3d> dataset = new ArrayList<>(length);
      Point3d min = new Point3d();
      Point3d max = new Point3d();

      min.sub(average, maxAmplitude);
      max.add(average, maxAmplitude);

      for (int i = 0; i < length; i++)
         dataset.add(RandomTools.generateRandomPoint3d(random, min, max));

      return dataset;
   }

   /**
    * Using the actual formula of the covariance matrix, <a href="https://en.wikipedia.org/wiki/Principal_component_analysis"> here</a>.
    */
   private static DenseMatrix64F computeCovarianceMatrix(List<Point3d> dataset, boolean corrected)
   {
      DenseMatrix64F covariance = new DenseMatrix64F(3, 3);
      int n = dataset.size();
      DenseMatrix64F datasetMatrix = new DenseMatrix64F(n, 3);

      Point3d average = GeometryTools.averagePoint3ds(dataset);

      for (int i = 0; i < n; i++)
      {
         Point3d dataPoint = dataset.get(i);
         datasetMatrix.set(i, 0, dataPoint.getX() - average.getX());
         datasetMatrix.set(i, 1, dataPoint.getY() - average.getY());
         datasetMatrix.set(i, 2, dataPoint.getZ() - average.getZ());
      }

      CommonOps.multInner(datasetMatrix, covariance);

      if (corrected)
      {
         CommonOps.scale(1.0 / (double) (n - 1.0), covariance);
      }
      else
      {
         CommonOps.scale(1.0 / (double) n, covariance);
      }

      return covariance;
   }
}
