package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Arrays;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.simple.SimpleMatrix;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;

public class RigidBodyInertiaTest
{
   private final Random random = new Random(100L);

   private ReferenceFrame worldFrame;
   private ReferenceFrame frameB;
   private ReferenceFrame frameC;
   private ReferenceFrame rotatedOnlyFrame;

   private RigidBodyInertia inertia;


   @Before
   public void setUp() throws Exception
   {
      worldFrame = ReferenceFrame.constructAWorldFrame("worldFrame");
      frameB = new ReferenceFrame("B", worldFrame)
      {
         private static final long serialVersionUID = 1L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(1.0, 2.0, 3.0);
            RigidBodyTransform translation = new RigidBodyTransform();
            translation.setTranslation(new Vector3d(3.0, 4.0, 5.0));
            transformToParent.multiply(translation);
         }
      };

      frameC = new ReferenceFrame("C", frameB)
      {
         private static final long serialVersionUID = 1L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(1.0, 2.0, 3.0);
            RigidBodyTransform translation = new RigidBodyTransform();
            translation.setTranslation(new Vector3d(3.0, 4.0, 5.0));
            transformToParent.multiply(translation);
         }
      };

      rotatedOnlyFrame = new ReferenceFrame("rotatedOnly", worldFrame)
      {
         private static final long serialVersionUID = 1L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(1.0, 2.0, 3.0);
         }
      };

      frameB.update();
      frameC.update();
      rotatedOnlyFrame.update();

      inertia = new RigidBodyInertia(frameB, getRandomSymmetricPositiveDefiniteMatrix(), getRandomPositiveNumber());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeKineticCoEnergyNoFrameChange()
   {
      Twist twist = new Twist(frameB, worldFrame, frameB, getRandomVector(), getRandomVector());
      assertKineticCoEnergyFrameIndependent(twist, inertia);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testComputeKineticCoEnergyWithFrameChange()
   {
      Twist twist = new Twist(frameB, worldFrame, frameB, getRandomVector(), getRandomVector());

      twist.changeFrame(frameC);
      inertia.changeFrame(frameC);
      assertKineticCoEnergyFrameIndependent(twist, inertia);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testChangeFrame()
   {
      ReferenceFrame oldFrame = inertia.getExpressedInFrame();
      SimpleMatrix inertiaOld = new SimpleMatrix(6, 6);
      inertia.getMatrix(inertiaOld.getMatrix());

      inertia.changeFrame(frameC);
      DenseMatrix64F inertiaCheap = new DenseMatrix64F(6, 6);
      inertia.getMatrix(inertiaCheap);

      RigidBodyTransform newToOld = inertia.getExpressedInFrame().getTransformToDesiredFrame(oldFrame);
      SimpleMatrix newToOldAdjoint = SimpleMatrix.wrap(adjoint(newToOld));
      SimpleMatrix newToOldAdjointTranspose = newToOldAdjoint.transpose();
      SimpleMatrix inertiaExpensive = newToOldAdjointTranspose.mult(inertiaOld).mult(newToOldAdjoint);

      double epsilon = 1e-8;
      JUnitTools.assertMatrixEquals(inertiaCheap, inertiaExpensive.getMatrix(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testChangeFrameKineticCoEnergyConsistency()
   {
      // compute kinetic co-energy in frame B
      Twist twist = new Twist(frameB, worldFrame, frameB, getRandomVector(), getRandomVector());
      double kineticCoEnergyFrameB = inertia.computeKineticCoEnergy(twist);

      // change frames to frame C, check that the kinetic co-energy is the same
      twist.changeFrame(frameC);
      inertia.changeFrame(frameC);
      double kineticCoEnergyFrameC = inertia.computeKineticCoEnergy(twist);

      // assert that they are the same
      double epsilon = 1e-8;
      assertEquals(kineticCoEnergyFrameB, kineticCoEnergyFrameC, epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSantiyIfChangeFramePurelyRotational()
   {
      inertia = new RigidBodyInertia(rotatedOnlyFrame, getRandomSymmetricPositiveDefiniteMatrix(), getRandomPositiveNumber());
      Matrix3d massMomentOfInertiaBeforeChange = inertia.getMassMomentOfInertiaPartCopy();

      inertia.changeFrame(worldFrame);
      Matrix3d massMomentOfInertiaAfterChange = inertia.getMassMomentOfInertiaPartCopy();

      if (!inertia.isCrossPartZero())
      {
         fail("Inertia should still be expressed in a frame that has the CoM as its origin; hence the cross part should be zero.");
      }

      double epsilon = 1e-8;
      assertEigenValuesPositiveAndEqual(massMomentOfInertiaBeforeChange, massMomentOfInertiaAfterChange, epsilon);
   }

   private double getRandomPositiveNumber()
   {
      return random.nextDouble() + 0.5;    // to make absolutely sure that the mass will be strictly greater than zero
   }

   private Vector3d getRandomVector()
   {
      return new Vector3d(random.nextDouble() - 0.5, random.nextDouble() - 0.5, random.nextDouble() - 0.5);
   }

   private Matrix3d getRandomSymmetricPositiveDefiniteMatrix()
   {
      // Wikipedia: a Hermitian (special case: symmetric) matrix is positive definite if:
      // There exists a unique lower triangular matrix L, with strictly positive diagonal elements, that allows the factorization of M into M = LL*.

      Matrix3d lowerTriangular = new Matrix3d();
      lowerTriangular.setM00(getRandomPositiveNumber());

      lowerTriangular.setM10(random.nextDouble() - 0.5);
      lowerTriangular.setM11(getRandomPositiveNumber());

      lowerTriangular.setM20(random.nextDouble() - 0.5);
      lowerTriangular.setM21(random.nextDouble() - 0.5);
      lowerTriangular.setM22(getRandomPositiveNumber());

      Matrix3d ret = new Matrix3d(lowerTriangular);
      ret.mulTransposeRight(ret, lowerTriangular);

      checkIsSymmetric(ret, 1e-8);
      checkEigenValuesRealAndPositive(ret, 1e-10);

      return ret;
   }

   /**
    * @param matrix matrix to check for symmetry
    * @param epsilon numerical tolerance
    */
   private static void checkIsSymmetric(Matrix3d matrix, double epsilon)
   {
      assertEquals(0.0, matrix.getM01() - matrix.getM10(), epsilon);
      assertEquals(0.0, matrix.getM02() - matrix.getM20(), epsilon);
      assertEquals(0.0, matrix.getM12() - matrix.getM21(), epsilon);
   }

   private static void checkEigenValuesRealAndPositive(Matrix3d matrix, double epsilon)
   {
      DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, matrix, denseMatrix);
      EigenDecomposition<DenseMatrix64F> eig = DecompositionFactory.eig(3, false);
      eig.decompose(denseMatrix);

      for (int i = 0; i < eig.getNumberOfEigenvalues(); i++)
      {
         double eigImaginaryPart = eig.getEigenvalue(i).getImaginary();
         double eigRealPart = eig.getEigenvalue(i).getReal();

         assertEquals(0.0, eigImaginaryPart, epsilon);
         assertTrue(eigRealPart > 0.0);
      }
   }

   private static void assertEigenValuesPositiveAndEqual(Matrix3d matrix1, Matrix3d matrix2, double epsilon)
   {
      DenseMatrix64F denseMatrix1 = new DenseMatrix64F(3, 3);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, matrix1, denseMatrix1);

      DenseMatrix64F denseMatrix2 = new DenseMatrix64F(3, 3);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, matrix2, denseMatrix2);

      EigenDecomposition<DenseMatrix64F> eig1 = DecompositionFactory.eig(3, false);
      eig1.decompose(denseMatrix1);
      double[] eigRealParts1 = new double[3];
      for (int i = 0; i < eig1.getNumberOfEigenvalues(); i++)
         eigRealParts1[i] = eig1.getEigenvalue(i).getReal();
      Arrays.sort(eigRealParts1);

      EigenDecomposition<DenseMatrix64F> eig2 = DecompositionFactory.eig(3, false);
      eig2.decompose(denseMatrix2);
      double[] eigRealParts2 = new double[3];
      for (int i = 0; i < eig2.getNumberOfEigenvalues(); i++)
         eigRealParts2[i] = eig2.getEigenvalue(i).getReal();
      Arrays.sort(eigRealParts2);

      for (int i = 0; i < eig1.getNumberOfEigenvalues(); i++)
      {
         assertEquals(0.0, eig1.getEigenvalue(i).getImaginary(), epsilon);
         assertEquals(0.0, eig2.getEigenvalue(i).getImaginary(), epsilon);
         assertEquals(eigRealParts2[0], eigRealParts2[0], epsilon);
      }


   }

   private static void assertKineticCoEnergyFrameIndependent(Twist twist, RigidBodyInertia inertia)
   {
      double kineticCoEnergyCheap = inertia.computeKineticCoEnergy(twist);

      SimpleMatrix twistMatrix = SimpleMatrix.wrap(twist.toMatrix());
      SimpleMatrix inertiaMatrix = new SimpleMatrix(6, 6);
      inertia.getMatrix(inertiaMatrix.getMatrix());

      SimpleMatrix momentumMatrix = inertiaMatrix.mult(twistMatrix);
      SimpleMatrix twistMatrixTranspose = twistMatrix.transpose();
      SimpleMatrix kineticCoEnergyExpensive = twistMatrixTranspose.mult(momentumMatrix);

      double epsilon = 1e-8;
      assertEquals(kineticCoEnergyExpensive.get(0, 0), kineticCoEnergyCheap, epsilon);
   }

   private static DenseMatrix64F adjoint(RigidBodyTransform transform)
   {
      Matrix3d rotation = new Matrix3d();
      transform.getRotation(rotation);

      Vector3d translation = new Vector3d();
      transform.getTranslation(translation);

      Matrix3d translationTilde = new Matrix3d();
      MatrixTools.toTildeForm(translationTilde, translation);

      DenseMatrix64F rotationDense = new DenseMatrix64F(3, 3);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, rotation, rotationDense);

      DenseMatrix64F translationTildeDense = new DenseMatrix64F(3, 3);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, translationTilde, translationTildeDense);

      DenseMatrix64F ret = new DenseMatrix64F(6, 6);

      // upper left:
      MatrixTools.setMatrixBlock(ret, 0, 0, rotationDense, 0, 0, 3, 3, 1.0);

      // lower left:

      DenseMatrix64F lowerLeft = new DenseMatrix64F(translationTildeDense.getNumRows(), rotationDense.getNumCols());
      CommonOps.mult(translationTildeDense, rotationDense, lowerLeft);
      MatrixTools.setMatrixBlock(ret, 3, 0, lowerLeft, 0, 0, 3, 3, 1.0);

      // lower right:
      MatrixTools.setMatrixBlock(ret, 3, 3, rotationDense, 0, 0, 3, 3, 1.0);

      return ret;
   }
}
