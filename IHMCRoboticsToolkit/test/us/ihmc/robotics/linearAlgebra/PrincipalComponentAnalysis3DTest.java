package us.ihmc.robotics.linearAlgebra;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.random.RandomTools;

public class PrincipalComponentAnalysis3DTest
{
   private static final boolean DEBUG = true;
   private static final double EPSILON_HIGH_PRECISION = 5.0e-7;
   private static final double EPSILON_LOW_PRECISION = 2.0e-3;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWith1DData()
   {
      Random random = new Random(5516315L);

      for (int trialNumber = 0; trialNumber < 20; trialNumber++)
      {
         if (DEBUG)
         {
            System.out.println("----------- Iteration #" + trialNumber + " ---------------------------");
         }
         Point3D origin = RandomTools.generateRandomPoint(random, 1.0, 1.0, 1.0);
         Point3D expectedMean = new Point3D();
         double pointScatteringAmplitude = 5.0;
         Vector3D expectedPrincipalAxis = RandomTools.generateRandomVector(random, 1.0);
         double expectedVarianceAlongPrincipalAxis = 0.0;
         double expectedStandardDeviationAlongPrincipalAxis = 0.0;

         int numberOfPoints = RandomTools.generateRandomInt(random, 10, 500);

         List<Point3D> listOfPoints = new ArrayList<>();

         // Variables to make the 1D-analysis of the data.
         List<Double> pointCloud1D = new ArrayList<>();
         double mean1D = 0.0;

         Vector3D offsetFromOrigin = new Vector3D();

         for (int i = 0; i < numberOfPoints; i++)
         {
            double nextGaussian = pointScatteringAmplitude * random.nextGaussian();
            offsetFromOrigin.set(expectedPrincipalAxis);
            offsetFromOrigin.scale(nextGaussian);

            Point3D newPoint = new Point3D();
            newPoint.set(origin);
            newPoint.add(offsetFromOrigin);
            listOfPoints.add(newPoint);

            expectedMean.setX(expectedMean.getX() + newPoint.getX() / numberOfPoints);
            expectedMean.setY(expectedMean.getY() + newPoint.getY() / numberOfPoints);
            expectedMean.setZ(expectedMean.getZ() + newPoint.getZ() / numberOfPoints);

            pointCloud1D.add(nextGaussian);
            mean1D += nextGaussian / numberOfPoints;
         }

         for (int i = 0; i < numberOfPoints; i++)
         {
            expectedVarianceAlongPrincipalAxis += MathTools.square(pointCloud1D.get(i) - mean1D) / numberOfPoints;
         }
         expectedStandardDeviationAlongPrincipalAxis = Math.sqrt(expectedVarianceAlongPrincipalAxis);

         Point3D estimatedMean = new Point3D();
         Vector3D estimatedPrincipalAxis = new Vector3D();
         Vector3D estimatedSecondaryAxis = new Vector3D();
         Vector3D estimatedThirdAxis = new Vector3D();
         Vector3D estimatedVariance = new Vector3D();
         Vector3D estimatedStandardDeviation = new Vector3D();
         Vector3D estimatedScaledPrincipalVector = new Vector3D();
         Vector3D estimatedScaledSecondaryVector = new Vector3D();
         Vector3D estimatedScaledThirdVector = new Vector3D();

         PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
         pca.setPointCloud(listOfPoints);
         pca.compute();
         pca.getMean(estimatedMean);
         pca.getPrincipalVectors(estimatedPrincipalAxis, estimatedSecondaryAxis, estimatedThirdAxis);
         pca.getStandardDeviation(estimatedStandardDeviation);
         pca.getVariance(estimatedVariance);
         pca.getScaledPrincipalVectors(estimatedScaledPrincipalVector, estimatedScaledSecondaryVector, estimatedScaledThirdVector);

         if (estimatedPrincipalAxis.dot(expectedPrincipalAxis) < 0.0)
         {
            expectedPrincipalAxis.negate();
         }

         if (DEBUG)
         {
            Vector3D errorPrincipalAxis = new Vector3D();
            errorPrincipalAxis.sub(expectedPrincipalAxis, estimatedPrincipalAxis);
            System.out.println("Error for the principal axis: " + errorPrincipalAxis);
         }

         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedPrincipalAxis, EPSILON_HIGH_PRECISION));

         if (DEBUG)
         {
            Vector3D errorMean = new Vector3D();
            errorMean.sub(expectedMean, estimatedMean);
            System.out.println("Error for the mean: " + errorMean);
         }

         assertTrue(expectedMean.epsilonEquals(estimatedMean, EPSILON_HIGH_PRECISION));

         // Test orthogonality between the principal axes:
         assertEquals(0.0, estimatedPrincipalAxis.dot(estimatedSecondaryAxis), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedPrincipalAxis.dot(estimatedThirdAxis), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedSecondaryAxis.dot(estimatedThirdAxis), EPSILON_HIGH_PRECISION);

         if (DEBUG)
         {
            System.out.println();
            System.out.println("Esimated principal axis: " + estimatedPrincipalAxis);
            System.out.println("Esimated secondary axis: " + estimatedSecondaryAxis);
            System.out.println("Esimated third axis: " + estimatedThirdAxis);
            System.out.println();
            System.out.println("Estimated variance: " + estimatedVariance);
            System.out.println("Estimated standard deviation: " + estimatedStandardDeviation);
         }

         assertEquals(expectedStandardDeviationAlongPrincipalAxis, estimatedStandardDeviation.getX(), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedStandardDeviation.getY(), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedStandardDeviation.getZ(), EPSILON_HIGH_PRECISION);

         assertEquals(expectedVarianceAlongPrincipalAxis, estimatedVariance.getX(), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedVariance.getY(), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedVariance.getZ(), EPSILON_HIGH_PRECISION);

         assertEquals(expectedVarianceAlongPrincipalAxis, estimatedScaledPrincipalVector.length(), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedScaledSecondaryVector.length(), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedScaledThirdVector.length(), EPSILON_HIGH_PRECISION);

         estimatedScaledPrincipalVector.normalize();
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_HIGH_PRECISION));

         RotationMatrix rotationMatrix = new RotationMatrix();
         pca.getPrincipalFrameRotationMatrix(rotationMatrix);
         rotationMatrix.getColumn(0, estimatedPrincipalAxis);
         rotationMatrix.getColumn(1, estimatedSecondaryAxis);
         rotationMatrix.getColumn(2, estimatedThirdAxis);

         assertTrue(rotationMatrix.isRotationMatrix());
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedPrincipalAxis, EPSILON_HIGH_PRECISION));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testWith2DData()
   {
      Random random = new Random(5516315L);

      for (int trialNumber = 0; trialNumber < 20; trialNumber++)
      {
         if (DEBUG)
         {
            System.out.println("----------- Iteration #" + trialNumber + " ---------------------------");
         }
         Point3D origin = RandomTools.generateRandomPoint(random, 1.0, 1.0, 1.0);
         Point3D expectedMean = new Point3D();
         Vector2D pointScatteringAmplitude = new Vector2D(15.0, 1.0);
         Vector3D expectedPrincipalAxis = RandomTools.generateRandomVector(random, 1.0);
         // Build the secondary vector such as it is orthogonal to the principal axis
         Vector3D randomVector = RandomTools.generateRandomVector(random, 1.0);
         Vector3D expectedSecondaryAxis = new Vector3D();
         expectedSecondaryAxis.cross(expectedPrincipalAxis, randomVector);
         expectedSecondaryAxis.normalize();
         Vector3D expectedThirdAxis = new Vector3D();
         expectedThirdAxis.cross(expectedPrincipalAxis, expectedSecondaryAxis);

         Vector2D expectedVariance = new Vector2D();
         Vector2D expectedStandardDeviation = new Vector2D();

         int numberOfPoints = RandomTools.generateRandomInt(random, 5000, 10000);

         List<Point3D> listOfPoints = new ArrayList<>();

         // Variables to make the 2D-analysis of the data in the principal frame.
         List<Point2D> pointCloudProjectedOnPrincipalAxes = new ArrayList<>();
         Point2D meanOnPrincipalAxes = new Point2D();

         Vector3D offsetFromOrigin = new Vector3D();

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D newPoint = new Point3D();
            newPoint.set(origin);

            double nextGaussianForPrincipalAxis = pointScatteringAmplitude.getX() * (1.0 - 2.0 * random.nextDouble());
            offsetFromOrigin.set(expectedPrincipalAxis);
            offsetFromOrigin.scale(nextGaussianForPrincipalAxis);
            newPoint.add(offsetFromOrigin);

            double nextGaussianForSecondaryAxis = pointScatteringAmplitude.getY() * (1.0 - 2.0 * random.nextDouble());
            offsetFromOrigin.set(expectedSecondaryAxis);
            offsetFromOrigin.scale(nextGaussianForSecondaryAxis);
            newPoint.add(offsetFromOrigin);

            listOfPoints.add(newPoint);

            expectedMean.setX(expectedMean.getX() + newPoint.getX() / numberOfPoints);
            expectedMean.setY(expectedMean.getY() + newPoint.getY() / numberOfPoints);
            expectedMean.setZ(expectedMean.getZ() + newPoint.getZ() / numberOfPoints);
         }

         Point3D estimatedMean = new Point3D();
         Vector3D estimatedPrincipalAxis = new Vector3D();
         Vector3D estimatedSecondaryAxis = new Vector3D();
         Vector3D estimatedThirdAxis = new Vector3D();
         Vector3D estimatedVariance = new Vector3D();
         Vector3D estimatedStandardDeviation = new Vector3D();
         Vector3D estimatedScaledPrincipalVector = new Vector3D();
         Vector3D estimatedScaledSecondaryVector = new Vector3D();
         Vector3D estimatedScaledThirdVector = new Vector3D();

         PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
         pca.setPointCloud(listOfPoints);
         pca.compute();
         pca.getMean(estimatedMean);
         pca.getPrincipalVectors(estimatedPrincipalAxis, estimatedSecondaryAxis, estimatedThirdAxis);
         pca.getStandardDeviation(estimatedStandardDeviation);
         pca.getVariance(estimatedVariance);
         pca.getScaledPrincipalVectors(estimatedScaledPrincipalVector, estimatedScaledSecondaryVector, estimatedScaledThirdVector);

         // Compute the expected standard deviation and variance in the estimated principal frame
         for (int i = 0; i < numberOfPoints; i++)
         {
            Vector3D vectorToPoint = new Vector3D();
            vectorToPoint.set(listOfPoints.get(i));
            double dotProductOnPrincipalAxis = vectorToPoint.dot(estimatedPrincipalAxis);
            double dotProductOnSecondaryAxis = vectorToPoint.dot(estimatedSecondaryAxis);
            pointCloudProjectedOnPrincipalAxes.add(new Point2D(dotProductOnPrincipalAxis, dotProductOnSecondaryAxis));
            meanOnPrincipalAxes.setX(meanOnPrincipalAxes.getX() + dotProductOnPrincipalAxis / numberOfPoints);
            meanOnPrincipalAxes.setY(meanOnPrincipalAxes.getY() + dotProductOnSecondaryAxis / numberOfPoints);
         }

         for (int i = 0; i < numberOfPoints; i++)
         {
            expectedVariance.setX(expectedVariance.getX() + MathTools.square(pointCloudProjectedOnPrincipalAxes.get(i).getX() - meanOnPrincipalAxes.getX()) / numberOfPoints);
            expectedVariance.setY(expectedVariance.getY() + MathTools.square(pointCloudProjectedOnPrincipalAxes.get(i).getY() - meanOnPrincipalAxes.getY()) / numberOfPoints);
         }
         expectedStandardDeviation.setX(Math.sqrt(expectedVariance.getX()));
         expectedStandardDeviation.setY(Math.sqrt(expectedVariance.getY()));

         if (estimatedPrincipalAxis.dot(expectedPrincipalAxis) < 0.0)
            expectedPrincipalAxis.negate();
         if (estimatedSecondaryAxis.dot(expectedSecondaryAxis) < 0.0)
            expectedSecondaryAxis.negate();
         if (estimatedThirdAxis.dot(expectedThirdAxis) < 0.0)
            expectedThirdAxis.negate();

         if (DEBUG)
         {
            System.out.println("Expected principal axis: " + expectedPrincipalAxis);
            System.out.println("Expected secondary axis: " + expectedSecondaryAxis);
            System.out.println("Expected third axis: " + expectedThirdAxis);
            System.out.println();
            System.out.println("Expected variance: " + expectedVariance);
            System.out.println("Expected standard deviation: " + expectedStandardDeviation);
            System.out.println();

            Vector3D errorPrincipalAxis = new Vector3D();
            errorPrincipalAxis.sub(expectedPrincipalAxis, estimatedPrincipalAxis);
            System.out.println("Error magnitude for the principal axis: " + errorPrincipalAxis.length());

            Vector3D errorSecondaryAxis = new Vector3D();
            errorSecondaryAxis.sub(expectedSecondaryAxis, estimatedSecondaryAxis);
            System.out.println("Error magnitude for the secondary axis: " + errorSecondaryAxis.length());

            Vector3D errorThirdAxis = new Vector3D();
            errorThirdAxis.sub(expectedThirdAxis, estimatedThirdAxis);
            System.out.println("Error magnitude for the third axis: " + errorThirdAxis.length());

            Vector3D errorMean = new Vector3D();
            errorMean.sub(expectedMean, estimatedMean);
            System.out.println("Error for the mean: " + errorMean);

            System.out.println();
            System.out.println("Estimated principal axis: " + estimatedPrincipalAxis);
            System.out.println("Estimated secondary axis: " + estimatedSecondaryAxis);
            System.out.println("Estimated third axis: " + estimatedThirdAxis);
            System.out.println();
            System.out.println("Estimated variance: " + estimatedVariance);
            System.out.println("Estimated standard deviation: " + estimatedStandardDeviation);
         }

         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedPrincipalAxis, EPSILON_LOW_PRECISION));
         assertTrue(expectedSecondaryAxis.epsilonEquals(estimatedSecondaryAxis, EPSILON_LOW_PRECISION));
         assertTrue(expectedThirdAxis.epsilonEquals(estimatedThirdAxis, EPSILON_HIGH_PRECISION));

         assertTrue(expectedMean.epsilonEquals(estimatedMean, EPSILON_HIGH_PRECISION));

         // Test orthogonality between the principal axes:
         assertEquals(0.0, estimatedPrincipalAxis.dot(estimatedSecondaryAxis), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedPrincipalAxis.dot(estimatedThirdAxis), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedSecondaryAxis.dot(estimatedThirdAxis), EPSILON_HIGH_PRECISION);

         assertEquals(expectedStandardDeviation.getX(), estimatedStandardDeviation.getX(), EPSILON_HIGH_PRECISION);
         assertEquals(expectedStandardDeviation.getY(), estimatedStandardDeviation.getY(), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedStandardDeviation.getZ(), EPSILON_HIGH_PRECISION);

         assertEquals(expectedVariance.getX(), estimatedVariance.getX(), EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.getY(), estimatedVariance.getY(), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedVariance.getZ(), EPSILON_HIGH_PRECISION);

         assertEquals(expectedVariance.getX(), estimatedScaledPrincipalVector.length(), EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.getY(), estimatedScaledSecondaryVector.length(), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedScaledThirdVector.length(), EPSILON_HIGH_PRECISION);

         estimatedScaledPrincipalVector.normalize();
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_LOW_PRECISION));
         assertTrue(estimatedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_HIGH_PRECISION));
         estimatedScaledSecondaryVector.normalize();
         assertTrue(expectedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_LOW_PRECISION));
         assertTrue(estimatedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_HIGH_PRECISION));

         RotationMatrix rotationMatrix = new RotationMatrix();
         pca.getPrincipalFrameRotationMatrix(rotationMatrix);
         rotationMatrix.getColumn(0, estimatedPrincipalAxis);
         rotationMatrix.getColumn(1, estimatedSecondaryAxis);
         rotationMatrix.getColumn(2, estimatedThirdAxis);

         assertTrue(rotationMatrix.isRotationMatrix());
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedPrincipalAxis, EPSILON_LOW_PRECISION));
         assertTrue(expectedSecondaryAxis.epsilonEquals(estimatedSecondaryAxis, EPSILON_LOW_PRECISION));
         assertTrue(estimatedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_HIGH_PRECISION));
         assertTrue(estimatedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_HIGH_PRECISION));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWith3DData()
   {
      Random random = new Random(5516315L);

      for (int trialNumber = 0; trialNumber < 1; trialNumber++)
      {
         if (DEBUG)
         {
            System.out.println("----------- Iteration #" + trialNumber + " ---------------------------");
         }
         Point3D origin = RandomTools.generateRandomPoint(random, 1.0, 1.0, 1.0);
         Point3D expectedMean = new Point3D();
         Vector3D pointScatteringAmplitude = new Vector3D(15.0, 1.0, 0.2);
         Vector3D expectedPrincipalAxis = RandomTools.generateRandomVector(random, 1.0);
         // Build the secondary vector such as it is orthogonal to the principal axis
         Vector3D randomVector = RandomTools.generateRandomVector(random, 1.0);
         Vector3D expectedSecondaryAxis = new Vector3D();
         expectedSecondaryAxis.cross(expectedPrincipalAxis, randomVector);
         expectedSecondaryAxis.normalize();
         Vector3D expectedThirdAxis = new Vector3D();
         expectedThirdAxis.cross(expectedPrincipalAxis, expectedSecondaryAxis);

         Vector3D expectedVariance = new Vector3D();
         Vector3D expectedStandardDeviation = new Vector3D();

         int numberOfPoints = RandomTools.generateRandomInt(random, 5000, 10000);

         List<Point3D> listOfPoints = new ArrayList<>();

         // Variables to make the 2D-analysis of the data in the principal frame.
         List<Point3D> pointCloudProjectedOnPrincipalAxes = new ArrayList<>();
         Point3D meanOnPrincipalAxes = new Point3D();

         Vector3D offsetFromOrigin = new Vector3D();

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D newPoint = new Point3D();
            newPoint.set(origin);

            double nextGaussianForPrincipalAxis = pointScatteringAmplitude.getX() * (1.0 - 2.0 * random.nextDouble());
            offsetFromOrigin.set(expectedPrincipalAxis);
            offsetFromOrigin.scale(nextGaussianForPrincipalAxis);
            newPoint.add(offsetFromOrigin);

            double nextGaussianForSecondaryAxis = pointScatteringAmplitude.getY() * (1.0 - 2.0 * random.nextDouble());
            offsetFromOrigin.set(expectedSecondaryAxis);
            offsetFromOrigin.scale(nextGaussianForSecondaryAxis);
            newPoint.add(offsetFromOrigin);

            double nextGaussianForThirdAxis = pointScatteringAmplitude.getZ() * (1.0 - 2.0 * random.nextDouble());
            offsetFromOrigin.set(expectedThirdAxis);
            offsetFromOrigin.scale(nextGaussianForThirdAxis);
            newPoint.add(offsetFromOrigin);

            listOfPoints.add(newPoint);

            expectedMean.setX(expectedMean.getX() + newPoint.getX() / numberOfPoints);
            expectedMean.setY(expectedMean.getY() + newPoint.getY() / numberOfPoints);
            expectedMean.setZ(expectedMean.getZ() + newPoint.getZ() / numberOfPoints);
         }

         Point3D estimatedMean = new Point3D();
         Vector3D estimatedPrincipalAxis = new Vector3D();
         Vector3D estimatedSecondaryAxis = new Vector3D();
         Vector3D estimatedThirdAxis = new Vector3D();
         Vector3D estimatedVariance = new Vector3D();
         Vector3D estimatedStandardDeviation = new Vector3D();
         Vector3D estimatedScaledPrincipalVector = new Vector3D();
         Vector3D estimatedScaledSecondaryVector = new Vector3D();
         Vector3D estimatedScaledThirdVector = new Vector3D();

         PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
         pca.setPointCloud(listOfPoints);
         pca.compute();
         pca.getMean(estimatedMean);
         pca.getPrincipalVectors(estimatedPrincipalAxis, estimatedSecondaryAxis, estimatedThirdAxis);
         pca.getStandardDeviation(estimatedStandardDeviation);
         pca.getVariance(estimatedVariance);
         pca.getScaledPrincipalVectors(estimatedScaledPrincipalVector, estimatedScaledSecondaryVector, estimatedScaledThirdVector);

         // Compute the expected standard deviation and variance in the estimated principal frame
         for (int i = 0; i < numberOfPoints; i++)
         {
            Vector3D vectorToPoint = new Vector3D();
            vectorToPoint.set(listOfPoints.get(i));
            double dotProductOnPrincipalAxis = vectorToPoint.dot(estimatedPrincipalAxis);
            double dotProductOnSecondaryAxis = vectorToPoint.dot(estimatedSecondaryAxis);
            double dotProductOnThirdAxis = vectorToPoint.dot(estimatedThirdAxis);
            pointCloudProjectedOnPrincipalAxes.add(new Point3D(dotProductOnPrincipalAxis, dotProductOnSecondaryAxis, dotProductOnThirdAxis));
            meanOnPrincipalAxes.setX(meanOnPrincipalAxes.getX() + dotProductOnPrincipalAxis / numberOfPoints);
            meanOnPrincipalAxes.setY(meanOnPrincipalAxes.getY() + dotProductOnSecondaryAxis / numberOfPoints);
            meanOnPrincipalAxes.setZ(meanOnPrincipalAxes.getZ() + dotProductOnThirdAxis / numberOfPoints);
         }

         for (int i = 0; i < numberOfPoints; i++)
         {
            expectedVariance.setX(expectedVariance.getX() + MathTools.square(pointCloudProjectedOnPrincipalAxes.get(i).getX() - meanOnPrincipalAxes.getX()) / numberOfPoints);
            expectedVariance.setY(expectedVariance.getY() + MathTools.square(pointCloudProjectedOnPrincipalAxes.get(i).getY() - meanOnPrincipalAxes.getY()) / numberOfPoints);
            expectedVariance.setZ(expectedVariance.getZ() + MathTools.square(pointCloudProjectedOnPrincipalAxes.get(i).getZ() - meanOnPrincipalAxes.getZ()) / numberOfPoints);
         }
         expectedStandardDeviation.setX(Math.sqrt(expectedVariance.getX()));
         expectedStandardDeviation.setY(Math.sqrt(expectedVariance.getY()));
         expectedStandardDeviation.setZ(Math.sqrt(expectedVariance.getZ()));

         if (estimatedPrincipalAxis.dot(expectedPrincipalAxis) < 0.0)
            expectedPrincipalAxis.negate();
         if (estimatedSecondaryAxis.dot(expectedSecondaryAxis) < 0.0)
            expectedSecondaryAxis.negate();
         if (estimatedThirdAxis.dot(expectedThirdAxis) < 0.0)
            expectedThirdAxis.negate();

         if (DEBUG)
         {
            System.out.println("Expected principal axis: " + expectedPrincipalAxis);
            System.out.println("Expected secondary axis: " + expectedSecondaryAxis);
            System.out.println("Expected third axis: " + expectedThirdAxis);
            System.out.println();
            System.out.println("Expected variance: " + expectedVariance);
            System.out.println("Expected standard deviation: " + expectedStandardDeviation);
            System.out.println();

            Vector3D errorPrincipalAxis = new Vector3D();
            errorPrincipalAxis.sub(expectedPrincipalAxis, estimatedPrincipalAxis);
            System.out.println("Error magnitude for the principal axis: " + errorPrincipalAxis.length());

            Vector3D errorSecondaryAxis = new Vector3D();
            errorSecondaryAxis.sub(expectedSecondaryAxis, estimatedSecondaryAxis);
            System.out.println("Error magnitude for the secondary axis: " + errorSecondaryAxis.length());

            Vector3D errorThirdAxis = new Vector3D();
            errorThirdAxis.sub(expectedThirdAxis, estimatedThirdAxis);
            System.out.println("Error magnitude for the third axis: " + errorThirdAxis.length());

            Vector3D errorMean = new Vector3D();
            errorMean.sub(expectedMean, estimatedMean);
            System.out.println("Error for the mean: " + errorMean);

            System.out.println();
            System.out.println("Estimated principal axis: " + estimatedPrincipalAxis);
            System.out.println("Estimated secondary axis: " + estimatedSecondaryAxis);
            System.out.println("Estimated third axis: " + estimatedThirdAxis);
            System.out.println();
            System.out.println("Estimated variance: " + estimatedVariance);
            System.out.println("Estimated standard deviation: " + estimatedStandardDeviation);
         }

         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedPrincipalAxis, EPSILON_LOW_PRECISION));
         assertTrue(expectedSecondaryAxis.epsilonEquals(estimatedSecondaryAxis, EPSILON_LOW_PRECISION));
         assertTrue(expectedThirdAxis.epsilonEquals(estimatedThirdAxis, EPSILON_LOW_PRECISION));

         assertTrue(expectedMean.epsilonEquals(estimatedMean, EPSILON_HIGH_PRECISION));

         // Test orthogonality between the principal axes:
         assertEquals(0.0, estimatedPrincipalAxis.dot(estimatedSecondaryAxis), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedPrincipalAxis.dot(estimatedThirdAxis), EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedSecondaryAxis.dot(estimatedThirdAxis), EPSILON_HIGH_PRECISION);

         assertEquals(expectedStandardDeviation.getX(), estimatedStandardDeviation.getX(), EPSILON_HIGH_PRECISION);
         assertEquals(expectedStandardDeviation.getY(), estimatedStandardDeviation.getY(), EPSILON_HIGH_PRECISION);
         assertEquals(expectedStandardDeviation.getZ(), estimatedStandardDeviation.getZ(), EPSILON_HIGH_PRECISION);

         assertEquals(expectedVariance.getX(), estimatedVariance.getX(), EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.getY(), estimatedVariance.getY(), EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.getZ(), estimatedVariance.getZ(), EPSILON_HIGH_PRECISION);

         assertEquals(expectedVariance.getX(), estimatedScaledPrincipalVector.length(), EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.getY(), estimatedScaledSecondaryVector.length(), EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.getZ(), estimatedScaledThirdVector.length(), EPSILON_HIGH_PRECISION);

         estimatedScaledPrincipalVector.normalize();
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_LOW_PRECISION));
         assertTrue(estimatedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_HIGH_PRECISION));
         estimatedScaledSecondaryVector.normalize();
         assertTrue(expectedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_LOW_PRECISION));
         assertTrue(estimatedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_HIGH_PRECISION));
         estimatedScaledThirdVector.normalize();
         assertTrue(expectedThirdAxis.epsilonEquals(estimatedScaledThirdVector, EPSILON_LOW_PRECISION));
         assertTrue(estimatedThirdAxis.epsilonEquals(estimatedScaledThirdVector, EPSILON_HIGH_PRECISION));

         RotationMatrix rotationMatrix = new RotationMatrix();
         pca.getPrincipalFrameRotationMatrix(rotationMatrix);
         rotationMatrix.getColumn(0, estimatedPrincipalAxis);
         rotationMatrix.getColumn(1, estimatedSecondaryAxis);
         rotationMatrix.getColumn(2, estimatedThirdAxis);

         assertTrue(rotationMatrix.isRotationMatrix());
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedPrincipalAxis, EPSILON_LOW_PRECISION));
         assertTrue(estimatedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_HIGH_PRECISION));
         assertTrue(expectedSecondaryAxis.epsilonEquals(estimatedSecondaryAxis, EPSILON_LOW_PRECISION));
         assertTrue(estimatedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_HIGH_PRECISION));
         assertTrue(expectedThirdAxis.epsilonEquals(estimatedThirdAxis, EPSILON_LOW_PRECISION));
         assertTrue(estimatedThirdAxis.epsilonEquals(estimatedScaledThirdVector, EPSILON_HIGH_PRECISION));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.FAST)
	@Test(timeout = 30000)
	/**
	 * Make sure PCA does not crap out if it gets an empty list of data points.
	 */
	public void testNoData()
	{
	   PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
	   ArrayList<Point3D> listOfPoints = new ArrayList<>();
	   pca.setPointCloud(listOfPoints);
	   pca.compute();
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.FAST)
	@Test(timeout = 30000)
	/**
	 * Make sure PCA does not crap out if a single data point is passed to it.
	 */
	public void testSingleDataPoint()
	{
	   Random random = new Random(1298490387L);

	   PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
	   ArrayList<Point3D> listOfPoints = new ArrayList<>();
	   listOfPoints.add(RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0));
	   pca.setPointCloud(listOfPoints);
	   pca.compute();
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.FAST)
	@Test(timeout = 30000)
	/**
	 * Make sure PCA does not crap out if two data points are passed to it.
	 */
	public void testTwoDataPoint()
	{
	   Random random = new Random(1298490387L);

	   PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
	   ArrayList<Point3D> listOfPoints = new ArrayList<>();
	   listOfPoints.add(RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0));
	   listOfPoints.add(RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0));
	   pca.setPointCloud(listOfPoints);
	   pca.compute();
	}

	@ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.FAST)
	@Test(timeout = 30000)
	/**
	 * Edge case:
	 * PCA used to fail if all data points are on the y axis. Make sure it returns the correct principal
	 * direction.
	 */
	public void testAllignedDataPointsOnY()
	{
	   Random random = new Random(129849038127L);
	   Vector3D direction = new Vector3D(0.0, 1.0, 0.0);
	   direction.normalize();

	   PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
	   ArrayList<Point3D> listOfPoints = new ArrayList<>();

	   for (int i = 0; i < 100; i++)
	   {
	      Point3D point = new Point3D(direction);
	      point.scale(5.0 * random.nextGaussian());
	      listOfPoints.add(point);
	   }

	   Vector3D estimatedPrincipalAxis = new Vector3D();
	   pca.setPointCloud(listOfPoints);
	   pca.compute();
	   pca.getPrincipalVector(estimatedPrincipalAxis);

	   assertEquals(estimatedPrincipalAxis.length(), 1.0, EPSILON_HIGH_PRECISION);
	   double dotProduct = Math.abs(estimatedPrincipalAxis.dot(direction));
	   assertEquals(1.0, dotProduct, EPSILON_HIGH_PRECISION);
	}
}
