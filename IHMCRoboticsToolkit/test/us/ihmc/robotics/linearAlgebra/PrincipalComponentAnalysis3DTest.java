package us.ihmc.robotics.linearAlgebra;

import org.junit.Test;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.random.RandomTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class PrincipalComponentAnalysis3DTest
{
   private static final boolean DEBUG = false;
   private static final double EPSILON_HIGH_PRECISION = 1.0e-12;
   private static final double EPSILON_LOW_PRECISION = 2.0e-3;

	@EstimatedDuration(duration = 0.0)
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
         Point3d origin = RandomTools.generateRandomPoint(random, 1.0, 1.0, 1.0);
         Point3d expectedMean = new Point3d();
         double pointScatteringAmplitude = 5.0;
         Vector3d expectedPrincipalAxis = RandomTools.generateRandomVector(random, 1.0);
         double expectedVarianceAlongPrincipalAxis = 0.0;
         double expectedStandardDeviationAlongPrincipalAxis = 0.0;

         int numberOfPoints = RandomTools.generateRandomInt(random, 10, 500);

         List<Point3d> listOfPoints = new ArrayList<>();

         // Variables to make the 1D-analysis of the data.
         List<Double> pointCloud1D = new ArrayList<>();
         double mean1D = 0.0;

         Vector3d offsetFromOrigin = new Vector3d();

         for (int i = 0; i < numberOfPoints; i++)
         {
            double nextGaussian = pointScatteringAmplitude * random.nextGaussian();
            offsetFromOrigin.set(expectedPrincipalAxis);
            offsetFromOrigin.scale(nextGaussian);

            Point3d newPoint = new Point3d();
            newPoint.set(origin);
            newPoint.add(offsetFromOrigin);
            listOfPoints.add(newPoint);

            expectedMean.x += newPoint.x / numberOfPoints;
            expectedMean.y += newPoint.y / numberOfPoints;
            expectedMean.z += newPoint.z / numberOfPoints;

            pointCloud1D.add(nextGaussian);
            mean1D += nextGaussian / numberOfPoints;
         }

         for (int i = 0; i < numberOfPoints; i++)
         {
            expectedVarianceAlongPrincipalAxis += MathTools.square(pointCloud1D.get(i) - mean1D) / numberOfPoints;
         }
         expectedStandardDeviationAlongPrincipalAxis = Math.sqrt(expectedVarianceAlongPrincipalAxis);

         Point3d estimatedMean = new Point3d();
         Vector3d estimatedPrincipalAxis = new Vector3d();
         Vector3d estimatedSecondaryAxis = new Vector3d();
         Vector3d estimatedThirdAxis = new Vector3d();
         Vector3d estimatedVariance = new Vector3d();
         Vector3d estimatedStandardDeviation = new Vector3d();
         Vector3d estimatedScaledPrincipalVector = new Vector3d();
         Vector3d estimatedScaledSecondaryVector = new Vector3d();
         Vector3d estimatedScaledThirdVector = new Vector3d();

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
            Vector3d errorPrincipalAxis = new Vector3d();
            errorPrincipalAxis.sub(expectedPrincipalAxis, estimatedPrincipalAxis);
            System.out.println("Error for the principal axis: " + errorPrincipalAxis);
         }

         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedPrincipalAxis, EPSILON_HIGH_PRECISION));

         if (DEBUG)
         {
            Vector3d errorMean = new Vector3d();
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

         assertEquals(expectedStandardDeviationAlongPrincipalAxis, estimatedStandardDeviation.x, EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedStandardDeviation.y, EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedStandardDeviation.z, EPSILON_HIGH_PRECISION);

         assertEquals(expectedVarianceAlongPrincipalAxis, estimatedVariance.x, EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedVariance.y, EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedVariance.z, EPSILON_HIGH_PRECISION);

         assertEquals(expectedVarianceAlongPrincipalAxis, MathTools.square(estimatedScaledPrincipalVector.length()) / numberOfPoints, EPSILON_HIGH_PRECISION);
         assertEquals(0.0, MathTools.square(estimatedScaledSecondaryVector.length()) / numberOfPoints, EPSILON_HIGH_PRECISION);
         assertEquals(0.0, MathTools.square(estimatedScaledThirdVector.length()) / numberOfPoints, EPSILON_HIGH_PRECISION);

         estimatedScaledPrincipalVector.normalize();
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_HIGH_PRECISION));

         Matrix3d rotationMatrix = new Matrix3d();
         pca.getPrincipalFrameRotationMatrix(rotationMatrix);
         rotationMatrix.getColumn(0, estimatedPrincipalAxis);
         rotationMatrix.getColumn(1, estimatedSecondaryAxis);
         rotationMatrix.getColumn(2, estimatedThirdAxis);

         assertTrue(RotationFunctions.isRotationProper(rotationMatrix));
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedPrincipalAxis, EPSILON_HIGH_PRECISION));
      }
   }

	@EstimatedDuration(duration = 0.1)
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
         Point3d origin = RandomTools.generateRandomPoint(random, 1.0, 1.0, 1.0);
         Point3d expectedMean = new Point3d();
         Vector2d pointScatteringAmplitude = new Vector2d(15.0, 1.0);
         Vector3d expectedPrincipalAxis = RandomTools.generateRandomVector(random, 1.0);
         // Build the secondary vector such as it is orthogonal to the principal axis
         Vector3d randomVector = RandomTools.generateRandomVector(random, 1.0);
         Vector3d expectedSecondaryAxis = new Vector3d();
         expectedSecondaryAxis.cross(expectedPrincipalAxis, randomVector);
         expectedSecondaryAxis.normalize();
         Vector3d expectedThirdAxis = new Vector3d();
         expectedThirdAxis.cross(expectedPrincipalAxis, expectedSecondaryAxis);

         Vector2d expectedVariance = new Vector2d();
         Vector2d expectedStandardDeviation = new Vector2d();

         int numberOfPoints = RandomTools.generateRandomInt(random, 5000, 10000);

         List<Point3d> listOfPoints = new ArrayList<>();

         // Variables to make the 2D-analysis of the data in the principal frame.
         List<Point2d> pointCloudProjectedOnPrincipalAxes = new ArrayList<>();
         Point2d meanOnPrincipalAxes = new Point2d();

         Vector3d offsetFromOrigin = new Vector3d();

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3d newPoint = new Point3d();
            newPoint.set(origin);

            double nextGaussianForPrincipalAxis = pointScatteringAmplitude.x * (1.0 - 2.0 * random.nextDouble());
            offsetFromOrigin.set(expectedPrincipalAxis);
            offsetFromOrigin.scale(nextGaussianForPrincipalAxis);
            newPoint.add(offsetFromOrigin);

            double nextGaussianForSecondaryAxis = pointScatteringAmplitude.y * (1.0 - 2.0 * random.nextDouble());
            offsetFromOrigin.set(expectedSecondaryAxis);
            offsetFromOrigin.scale(nextGaussianForSecondaryAxis);
            newPoint.add(offsetFromOrigin);

            listOfPoints.add(newPoint);

            expectedMean.x += newPoint.x / numberOfPoints;
            expectedMean.y += newPoint.y / numberOfPoints;
            expectedMean.z += newPoint.z / numberOfPoints;
         }

         Point3d estimatedMean = new Point3d();
         Vector3d estimatedPrincipalAxis = new Vector3d();
         Vector3d estimatedSecondaryAxis = new Vector3d();
         Vector3d estimatedThirdAxis = new Vector3d();
         Vector3d estimatedVariance = new Vector3d();
         Vector3d estimatedStandardDeviation = new Vector3d();
         Vector3d estimatedScaledPrincipalVector = new Vector3d();
         Vector3d estimatedScaledSecondaryVector = new Vector3d();
         Vector3d estimatedScaledThirdVector = new Vector3d();

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
            Vector3d vectorToPoint = new Vector3d();
            vectorToPoint.set(listOfPoints.get(i));
            double dotProductOnPrincipalAxis = vectorToPoint.dot(estimatedPrincipalAxis);
            double dotProductOnSecondaryAxis = vectorToPoint.dot(estimatedSecondaryAxis);
            pointCloudProjectedOnPrincipalAxes.add(new Point2d(dotProductOnPrincipalAxis, dotProductOnSecondaryAxis));
            meanOnPrincipalAxes.x += dotProductOnPrincipalAxis / numberOfPoints;
            meanOnPrincipalAxes.y += dotProductOnSecondaryAxis / numberOfPoints;
         }

         for (int i = 0; i < numberOfPoints; i++)
         {
            expectedVariance.x += MathTools.square(pointCloudProjectedOnPrincipalAxes.get(i).x - meanOnPrincipalAxes.x) / numberOfPoints;
            expectedVariance.y += MathTools.square(pointCloudProjectedOnPrincipalAxes.get(i).y - meanOnPrincipalAxes.y) / numberOfPoints;
         }
         expectedStandardDeviation.x = Math.sqrt(expectedVariance.x);
         expectedStandardDeviation.y = Math.sqrt(expectedVariance.y);

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

            Vector3d errorPrincipalAxis = new Vector3d();
            errorPrincipalAxis.sub(expectedPrincipalAxis, estimatedPrincipalAxis);
            System.out.println("Error magnitude for the principal axis: " + errorPrincipalAxis.length());

            Vector3d errorSecondaryAxis = new Vector3d();
            errorSecondaryAxis.sub(expectedSecondaryAxis, estimatedSecondaryAxis);
            System.out.println("Error magnitude for the secondary axis: " + errorSecondaryAxis.length());

            Vector3d errorThirdAxis = new Vector3d();
            errorThirdAxis.sub(expectedThirdAxis, estimatedThirdAxis);
            System.out.println("Error magnitude for the third axis: " + errorThirdAxis.length());

            Vector3d errorMean = new Vector3d();
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

         assertEquals(expectedStandardDeviation.x, estimatedStandardDeviation.x, EPSILON_HIGH_PRECISION);
         assertEquals(expectedStandardDeviation.y, estimatedStandardDeviation.y, EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedStandardDeviation.z, EPSILON_HIGH_PRECISION);

         assertEquals(expectedVariance.x, estimatedVariance.x, EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.y, estimatedVariance.y, EPSILON_HIGH_PRECISION);
         assertEquals(0.0, estimatedVariance.z, EPSILON_HIGH_PRECISION);

         assertEquals(expectedVariance.x, MathTools.square(estimatedScaledPrincipalVector.length()) / numberOfPoints, EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.y, MathTools.square(estimatedScaledSecondaryVector.length()) / numberOfPoints, EPSILON_HIGH_PRECISION);
         assertEquals(0.0, MathTools.square(estimatedScaledThirdVector.length()) / numberOfPoints, EPSILON_HIGH_PRECISION);

         estimatedScaledPrincipalVector.normalize();
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_LOW_PRECISION));
         assertTrue(estimatedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_HIGH_PRECISION));
         estimatedScaledSecondaryVector.normalize();
         assertTrue(expectedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_LOW_PRECISION));
         assertTrue(estimatedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_HIGH_PRECISION));

         Matrix3d rotationMatrix = new Matrix3d();
         pca.getPrincipalFrameRotationMatrix(rotationMatrix);
         rotationMatrix.getColumn(0, estimatedPrincipalAxis);
         rotationMatrix.getColumn(1, estimatedSecondaryAxis);
         rotationMatrix.getColumn(2, estimatedThirdAxis);

         assertTrue(RotationFunctions.isRotationProper(rotationMatrix));
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedPrincipalAxis, EPSILON_LOW_PRECISION));
         assertTrue(expectedSecondaryAxis.epsilonEquals(estimatedSecondaryAxis, EPSILON_LOW_PRECISION));
         assertTrue(estimatedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_HIGH_PRECISION));
         assertTrue(estimatedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_HIGH_PRECISION));
      }
   }

	@EstimatedDuration(duration = 0.0)
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
         Point3d origin = RandomTools.generateRandomPoint(random, 1.0, 1.0, 1.0);
         Point3d expectedMean = new Point3d();
         Vector3d pointScatteringAmplitude = new Vector3d(15.0, 1.0, 0.2);
         Vector3d expectedPrincipalAxis = RandomTools.generateRandomVector(random, 1.0);
         // Build the secondary vector such as it is orthogonal to the principal axis
         Vector3d randomVector = RandomTools.generateRandomVector(random, 1.0);
         Vector3d expectedSecondaryAxis = new Vector3d();
         expectedSecondaryAxis.cross(expectedPrincipalAxis, randomVector);
         expectedSecondaryAxis.normalize();
         Vector3d expectedThirdAxis = new Vector3d();
         expectedThirdAxis.cross(expectedPrincipalAxis, expectedSecondaryAxis);

         Vector3d expectedVariance = new Vector3d();
         Vector3d expectedStandardDeviation = new Vector3d();

         int numberOfPoints = RandomTools.generateRandomInt(random, 5000, 10000);

         List<Point3d> listOfPoints = new ArrayList<>();

         // Variables to make the 2D-analysis of the data in the principal frame.
         List<Point3d> pointCloudProjectedOnPrincipalAxes = new ArrayList<>();
         Point3d meanOnPrincipalAxes = new Point3d();

         Vector3d offsetFromOrigin = new Vector3d();

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3d newPoint = new Point3d();
            newPoint.set(origin);

            double nextGaussianForPrincipalAxis = pointScatteringAmplitude.x * (1.0 - 2.0 * random.nextDouble());
            offsetFromOrigin.set(expectedPrincipalAxis);
            offsetFromOrigin.scale(nextGaussianForPrincipalAxis);
            newPoint.add(offsetFromOrigin);

            double nextGaussianForSecondaryAxis = pointScatteringAmplitude.y * (1.0 - 2.0 * random.nextDouble());
            offsetFromOrigin.set(expectedSecondaryAxis);
            offsetFromOrigin.scale(nextGaussianForSecondaryAxis);
            newPoint.add(offsetFromOrigin);

            double nextGaussianForThirdAxis = pointScatteringAmplitude.z * (1.0 - 2.0 * random.nextDouble());
            offsetFromOrigin.set(expectedThirdAxis);
            offsetFromOrigin.scale(nextGaussianForThirdAxis);
            newPoint.add(offsetFromOrigin);

            listOfPoints.add(newPoint);

            expectedMean.x += newPoint.x / numberOfPoints;
            expectedMean.y += newPoint.y / numberOfPoints;
            expectedMean.z += newPoint.z / numberOfPoints;
         }

         Point3d estimatedMean = new Point3d();
         Vector3d estimatedPrincipalAxis = new Vector3d();
         Vector3d estimatedSecondaryAxis = new Vector3d();
         Vector3d estimatedThirdAxis = new Vector3d();
         Vector3d estimatedVariance = new Vector3d();
         Vector3d estimatedStandardDeviation = new Vector3d();
         Vector3d estimatedScaledPrincipalVector = new Vector3d();
         Vector3d estimatedScaledSecondaryVector = new Vector3d();
         Vector3d estimatedScaledThirdVector = new Vector3d();

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
            Vector3d vectorToPoint = new Vector3d();
            vectorToPoint.set(listOfPoints.get(i));
            double dotProductOnPrincipalAxis = vectorToPoint.dot(estimatedPrincipalAxis);
            double dotProductOnSecondaryAxis = vectorToPoint.dot(estimatedSecondaryAxis);
            double dotProductOnThirdAxis = vectorToPoint.dot(estimatedThirdAxis);
            pointCloudProjectedOnPrincipalAxes.add(new Point3d(dotProductOnPrincipalAxis, dotProductOnSecondaryAxis, dotProductOnThirdAxis));
            meanOnPrincipalAxes.x += dotProductOnPrincipalAxis / numberOfPoints;
            meanOnPrincipalAxes.y += dotProductOnSecondaryAxis / numberOfPoints;
            meanOnPrincipalAxes.z += dotProductOnThirdAxis / numberOfPoints;
         }

         for (int i = 0; i < numberOfPoints; i++)
         {
            expectedVariance.x += MathTools.square(pointCloudProjectedOnPrincipalAxes.get(i).x - meanOnPrincipalAxes.x) / numberOfPoints;
            expectedVariance.y += MathTools.square(pointCloudProjectedOnPrincipalAxes.get(i).y - meanOnPrincipalAxes.y) / numberOfPoints;
            expectedVariance.z += MathTools.square(pointCloudProjectedOnPrincipalAxes.get(i).z - meanOnPrincipalAxes.z) / numberOfPoints;
         }
         expectedStandardDeviation.x = Math.sqrt(expectedVariance.x);
         expectedStandardDeviation.y = Math.sqrt(expectedVariance.y);
         expectedStandardDeviation.z = Math.sqrt(expectedVariance.z);

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

            Vector3d errorPrincipalAxis = new Vector3d();
            errorPrincipalAxis.sub(expectedPrincipalAxis, estimatedPrincipalAxis);
            System.out.println("Error magnitude for the principal axis: " + errorPrincipalAxis.length());

            Vector3d errorSecondaryAxis = new Vector3d();
            errorSecondaryAxis.sub(expectedSecondaryAxis, estimatedSecondaryAxis);
            System.out.println("Error magnitude for the secondary axis: " + errorSecondaryAxis.length());

            Vector3d errorThirdAxis = new Vector3d();
            errorThirdAxis.sub(expectedThirdAxis, estimatedThirdAxis);
            System.out.println("Error magnitude for the third axis: " + errorThirdAxis.length());

            Vector3d errorMean = new Vector3d();
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

         assertEquals(expectedStandardDeviation.x, estimatedStandardDeviation.x, EPSILON_HIGH_PRECISION);
         assertEquals(expectedStandardDeviation.y, estimatedStandardDeviation.y, EPSILON_HIGH_PRECISION);
         assertEquals(expectedStandardDeviation.z, estimatedStandardDeviation.z, EPSILON_HIGH_PRECISION);

         assertEquals(expectedVariance.x, estimatedVariance.x, EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.y, estimatedVariance.y, EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.z, estimatedVariance.z, EPSILON_HIGH_PRECISION);

         assertEquals(expectedVariance.x, MathTools.square(estimatedScaledPrincipalVector.length()) / numberOfPoints, EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.y, MathTools.square(estimatedScaledSecondaryVector.length()) / numberOfPoints, EPSILON_HIGH_PRECISION);
         assertEquals(expectedVariance.z, MathTools.square(estimatedScaledThirdVector.length()) / numberOfPoints, EPSILON_HIGH_PRECISION);

         estimatedScaledPrincipalVector.normalize();
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_LOW_PRECISION));
         assertTrue(estimatedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_HIGH_PRECISION));
         estimatedScaledSecondaryVector.normalize();
         assertTrue(expectedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_LOW_PRECISION));
         assertTrue(estimatedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_HIGH_PRECISION));
         estimatedScaledThirdVector.normalize();
         assertTrue(expectedThirdAxis.epsilonEquals(estimatedScaledThirdVector, EPSILON_LOW_PRECISION));
         assertTrue(estimatedThirdAxis.epsilonEquals(estimatedScaledThirdVector, EPSILON_HIGH_PRECISION));

         Matrix3d rotationMatrix = new Matrix3d();
         pca.getPrincipalFrameRotationMatrix(rotationMatrix);
         rotationMatrix.getColumn(0, estimatedPrincipalAxis);
         rotationMatrix.getColumn(1, estimatedSecondaryAxis);
         rotationMatrix.getColumn(2, estimatedThirdAxis);

         assertTrue(RotationFunctions.isRotationProper(rotationMatrix));
         assertTrue(expectedPrincipalAxis.epsilonEquals(estimatedPrincipalAxis, EPSILON_LOW_PRECISION));
         assertTrue(estimatedPrincipalAxis.epsilonEquals(estimatedScaledPrincipalVector, EPSILON_HIGH_PRECISION));
         assertTrue(expectedSecondaryAxis.epsilonEquals(estimatedSecondaryAxis, EPSILON_LOW_PRECISION));
         assertTrue(estimatedSecondaryAxis.epsilonEquals(estimatedScaledSecondaryVector, EPSILON_HIGH_PRECISION));
         assertTrue(expectedThirdAxis.epsilonEquals(estimatedThirdAxis, EPSILON_LOW_PRECISION));
         assertTrue(estimatedThirdAxis.epsilonEquals(estimatedScaledThirdVector, EPSILON_HIGH_PRECISION));
      }
   }
}
