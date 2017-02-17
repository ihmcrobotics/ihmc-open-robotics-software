package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameMatrix3DTest
{
   private static final double EPSILON = 1e-10;
   private static final Random random = new Random(21651651L);
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame aFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("aFrame", worldFrame,
                                                                                                             EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));
   private static final ReferenceFrame bFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("bFrame", worldFrame,
                                                                                                             EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructors()
   {
      Matrix3D matrixTested = new Matrix3D();
      Matrix3D matrixExpected = new Matrix3D();

      FrameMatrix3D frameMatrix3D;

      frameMatrix3D = new FrameMatrix3D();

      matrixExpected.setToZero();
      frameMatrix3D.getMatrix(matrixTested);
      assertTrue(frameMatrix3D.getReferenceFrame() == worldFrame);
      assertTrue(matrixExpected.epsilonEquals(matrixTested, EPSILON));

      matrixExpected = RandomTools.generateRandomMatrix3d(random, 100.0);
      frameMatrix3D = new FrameMatrix3D(aFrame, matrixExpected);
      frameMatrix3D.getMatrix(matrixTested);
      assertTrue(frameMatrix3D.getReferenceFrame() == aFrame);
      assertTrue(matrixExpected.epsilonEquals(matrixTested, EPSILON));

      FrameMatrix3D secondFrameMatrix3D = new FrameMatrix3D(frameMatrix3D);
      frameMatrix3D.getMatrix(matrixExpected);
      secondFrameMatrix3D.getMatrix(matrixTested);
      assertTrue(secondFrameMatrix3D.getReferenceFrame() == frameMatrix3D.getReferenceFrame());
      assertTrue(matrixExpected.epsilonEquals(matrixTested, EPSILON));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetters()
   {
      Matrix3D matrixTested = new Matrix3D();
      Matrix3D matrixExpected = RandomTools.generateRandomDiagonalMatrix3d(random);

      FrameMatrix3D frameMatrix3D = new FrameMatrix3D(aFrame, RandomTools.generateRandomMatrix3d(random, 100.0));

      frameMatrix3D.set(matrixExpected);
      frameMatrix3D.getMatrix(matrixTested);

      assertTrue(frameMatrix3D.getReferenceFrame() == aFrame);
      assertTrue(matrixExpected.epsilonEquals(matrixTested, EPSILON));

      frameMatrix3D.setIncludingFrame(worldFrame, matrixExpected);
      frameMatrix3D.getMatrix(matrixTested);

      assertTrue(frameMatrix3D.getReferenceFrame() == worldFrame);
      assertTrue(matrixExpected.epsilonEquals(matrixTested, EPSILON));

      FrameMatrix3D secondFrameMatrix3D = new FrameMatrix3D(aFrame, RandomTools.generateRandomMatrix3d(random, 100.0));

      try
      {
         frameMatrix3D.set(secondFrameMatrix3D);
         fail("Should have thrown a ReferenceFrameMismatchException");
      }
      catch (ReferenceFrameMismatchException e)
      {
         // Good
      }

      frameMatrix3D.setIncludingFrame(secondFrameMatrix3D);
      frameMatrix3D.getMatrix(matrixExpected);
      secondFrameMatrix3D.getMatrix(matrixTested);
      assertTrue(frameMatrix3D.getReferenceFrame() == secondFrameMatrix3D.getReferenceFrame());
      assertTrue(matrixExpected.epsilonEquals(matrixTested, EPSILON));

      secondFrameMatrix3D.set(RandomTools.generateRandomMatrix3d(random, 100.0));
      frameMatrix3D.set(secondFrameMatrix3D);
      frameMatrix3D.getMatrix(matrixExpected);
      secondFrameMatrix3D.getMatrix(matrixTested);
      assertTrue(frameMatrix3D.getReferenceFrame() == secondFrameMatrix3D.getReferenceFrame());
      assertTrue(matrixExpected.epsilonEquals(matrixTested, EPSILON));

      matrixExpected = RandomTools.generateRandomMatrix3d(random, 10.0);
      frameMatrix3D = new FrameMatrix3D();
      frameMatrix3D.setM00(matrixExpected.getM00());
      frameMatrix3D.setM01(matrixExpected.getM01());
      frameMatrix3D.setM02(matrixExpected.getM02());
      frameMatrix3D.setM10(matrixExpected.getM10());
      frameMatrix3D.setM11(matrixExpected.getM11());
      frameMatrix3D.setM12(matrixExpected.getM12());
      frameMatrix3D.setM20(matrixExpected.getM20());
      frameMatrix3D.setM21(matrixExpected.getM21());
      frameMatrix3D.setM22(matrixExpected.getM22());
      frameMatrix3D.getMatrix(matrixTested);
      assertTrue(matrixExpected.epsilonEquals(matrixTested, EPSILON));

      matrixExpected = RandomTools.generateRandomMatrix3d(random, 10.0);
      frameMatrix3D = new FrameMatrix3D();
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            frameMatrix3D.setElement(row, column, matrixExpected.getElement(row, column));
         }
      }
      frameMatrix3D.getMatrix(matrixTested);
      assertTrue(matrixExpected.epsilonEquals(matrixTested, EPSILON));

      matrixExpected = RandomTools.generateRandomMatrix3d(random, 10.0);
      frameMatrix3D = new FrameMatrix3D();
      for (int row = 0; row < 3; row++)
      {
         Vector3D rowElements = new Vector3D();
         matrixExpected.getRow(row, rowElements);
         frameMatrix3D.setRow(row, rowElements.getX(), rowElements.getY(), rowElements.getZ());
      }
      frameMatrix3D.getMatrix(matrixTested);
      assertTrue(matrixExpected.epsilonEquals(matrixTested, EPSILON));

      matrixExpected = RandomTools.generateRandomMatrix3d(random, 10.0);
      frameMatrix3D = new FrameMatrix3D();
      for (int column = 0; column < 3; column++)
      {
         Vector3D columnElements = new Vector3D();
         matrixExpected.getColumn(column, columnElements);
         frameMatrix3D.setColumn(column, columnElements.getX(), columnElements.getY(), columnElements.getZ());
      }
      frameMatrix3D.getMatrix(matrixTested);
      assertTrue(matrixExpected.epsilonEquals(matrixTested, EPSILON));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetters()
   {
      FrameMatrix3D frameMatrix3D = new FrameMatrix3D();
      Matrix3D matrixExpected = RandomTools.generateRandomMatrix3d(random, 10.0);
      frameMatrix3D.setIncludingFrame(aFrame, matrixExpected);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertEquals(matrixExpected.getElement(row, column), frameMatrix3D.getElement(row, column), EPSILON);
         }
      }

      RotationMatrix matrixTested = new RotationMatrix();
      frameMatrix3D.getMatrix(matrixTested);
      assertTrue(matrixTested.epsilonEquals(matrixExpected, EPSILON));

      DenseMatrix64F denseMatrixTested = new DenseMatrix64F(3, 3);
      frameMatrix3D.getDenseMatrix(denseMatrixTested);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertEquals(matrixExpected.getElement(row, column), denseMatrixTested.get(row, column), EPSILON);
         }
      }

      denseMatrixTested.reshape(6, 6);
      int startRow = 3;
      int startColumn = 3;
      frameMatrix3D.getDenseMatrix(denseMatrixTested, startRow, startColumn);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertEquals(matrixExpected.getElement(row, column), denseMatrixTested.get(row + startRow, column + startColumn), EPSILON);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToNaN()
   {
      FrameMatrix3D frameMatrix3D = new FrameMatrix3D();
      frameMatrix3D.setIncludingFrame(aFrame, RandomTools.generateRandomMatrix3d(random, 10.0));
      frameMatrix3D.setToNaN();

      assertTrue(frameMatrix3D.getReferenceFrame() == aFrame);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(Double.isNaN(frameMatrix3D.getElement(row, column)));
         }
      }

      frameMatrix3D.setToNaN(bFrame);

      assertTrue(frameMatrix3D.getReferenceFrame() == bFrame);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(Double.isNaN(frameMatrix3D.getElement(row, column)));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToZero()
   {
      FrameMatrix3D frameMatrix3D = new FrameMatrix3D();
      frameMatrix3D.setIncludingFrame(aFrame, RandomTools.generateRandomMatrix3d(random, 10.0));
      frameMatrix3D.setToZero();

      assertTrue(frameMatrix3D.getReferenceFrame() == aFrame);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(frameMatrix3D.getElement(row, column) == 0.0);
         }
      }

      frameMatrix3D.setToZero(bFrame);

      assertTrue(frameMatrix3D.getReferenceFrame() == bFrame);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(frameMatrix3D.getElement(row, column) == 0.0);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetToIdentity()
   {
      FrameMatrix3D frameMatrix3D = new FrameMatrix3D();
      frameMatrix3D.setIncludingFrame(aFrame, RandomTools.generateRandomMatrix3d(random, 10.0));
      frameMatrix3D.setToIdentity();

      assertTrue(frameMatrix3D.getReferenceFrame() == aFrame);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            if (row != column)
               assertTrue(frameMatrix3D.getElement(row, column) == 0.0);
            else
               assertTrue(frameMatrix3D.getElement(row, column) == 1.0);
         }
      }

      frameMatrix3D.setToIdentity(bFrame);

      assertTrue(frameMatrix3D.getReferenceFrame() == bFrame);
      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            if (row != column)
               assertTrue(frameMatrix3D.getElement(row, column) == 0.0);
            else
               assertTrue(frameMatrix3D.getElement(row, column) == 1.0);
         }
      }
   }

   /**
    * Check that changing frame applies the expected transformation to the matrix3d held in
    * FrameMatrix3D
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testChangeFrame()
   {
      for (int i = 0; i < 1000; i++)
      {
         Matrix3D matrixTested = new Matrix3D();
         Matrix3D matrixOriginal = RandomTools.generateRandomDiagonalMatrix3d(random);
         Matrix3D matrixTransformed = new Matrix3D();

         ReferenceFrame randomFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("randomFrame" + i, ReferenceFrame.getWorldFrame(),
                                                                                                   EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));
         FrameMatrix3D frameMatrix3D = new FrameMatrix3D(randomFrame, matrixOriginal);
         frameMatrix3D.changeFrame(bFrame);

         RigidBodyTransform transformToWorldFrame = randomFrame.getTransformToDesiredFrame(bFrame);
         RotationMatrix rotationToWorld = new RotationMatrix();
         transformToWorldFrame.getRotation(rotationToWorld);
         rotationToWorld.transform(matrixOriginal, matrixTransformed);

         frameMatrix3D.getMatrix(matrixTested);
         assertTrue(frameMatrix3D.getReferenceFrame() == bFrame);
         assertTrue(matrixTransformed.epsilonEquals(matrixTested, EPSILON));
      }
   }

   /**
    * Test the changeFrame method by check that transforming a random vector in two different frames
    * using the FrameMatrix3D ends up being the same.
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testChangeFrameByTransformingAVectorInTwoDifferentFrames()
   {
      PoseReferenceFrame randomFrameA = new PoseReferenceFrame("randomFrameA", worldFrame);
      PoseReferenceFrame randomFrameB = new PoseReferenceFrame("randomFrameB", worldFrame);
      FrameVector originalVector = new FrameVector();
      FrameVector vectorTransformedInFrameA = new FrameVector();
      FrameVector vectorTransformedInFrameB = new FrameVector();

      FrameMatrix3D transformationMatrixToBeTested = new FrameMatrix3D();

      for (int i = 0; i < 1000; i++)
      {
         randomFrameA.setPoseAndUpdate(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));
         randomFrameB.setPoseAndUpdate(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));
         originalVector.setIncludingFrame(randomFrameA, RandomTools.generateRandomVector(random, 1.0));
         transformationMatrixToBeTested.setIncludingFrame(randomFrameA, RandomTools.generateRandomMatrix3d(random, 1.0));

         transformationMatrixToBeTested.transform(originalVector, vectorTransformedInFrameA);

         originalVector.changeFrame(randomFrameB);
         transformationMatrixToBeTested.changeFrame(randomFrameB);
         transformationMatrixToBeTested.transform(originalVector, vectorTransformedInFrameB);
         vectorTransformedInFrameB.changeFrame(randomFrameA);

         assertTrue(vectorTransformedInFrameB.epsilonEquals(vectorTransformedInFrameA, EPSILON));
      }

   }
}
