package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public abstract class DampedNullspaceCalculatorTest extends NullspaceCalculatorTest
{
   public abstract DampedNullspaceCalculator getDampedNullspaceProjectorCalculator();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleNullspaceProjectorWithDamping()
   {
      DampedNullspaceCalculator nullspaceCalculator = getDampedNullspaceProjectorCalculator();

      nullspaceCalculator.setPseudoInverseAlpha(0.1);

      DenseMatrix64F jacobian = new DenseMatrix64F(4, 2);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 5.0);
      jacobian.set(1, 0, 2.0);
      jacobian.set(1, 1, 6.0);
      jacobian.set(2, 0, 3.0);
      jacobian.set(2, 1, 7.0);
      jacobian.set(3, 0, 4.0);
      jacobian.set(3, 1, 8.0);

      DenseMatrix64F nullspaceProjector = new DenseMatrix64F(2, 2);
      nullspaceCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      DenseMatrix64F nullspaceProjectorExpected = new DenseMatrix64F(2, 2);
      nullspaceProjectorExpected.set(0, 0, 0.00540336);
      nullspaceProjectorExpected.set(0, 1,-0.00217364);
      nullspaceProjectorExpected.set(1, 0,-0.00217364);
      nullspaceProjectorExpected.set(1, 1, 0.000931872);

      for (int i = 0; i < nullspaceProjector.getNumElements(); i++)
         assertEquals(nullspaceProjectorExpected.get(i), nullspaceProjector.get(i), 1e-7);


      jacobian = new DenseMatrix64F(2, 2);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 1, 9.0);


      nullspaceProjector = new DenseMatrix64F(2, 2);
      nullspaceCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      nullspaceProjectorExpected = new DenseMatrix64F(2, 2);

      nullspaceProjectorExpected.set(0, 0, 0.0061905);
      nullspaceProjectorExpected.set(0, 1,-0.0045392);
      nullspaceProjectorExpected.set(1, 0,-0.0045392);
      nullspaceProjectorExpected.set(1, 1, 0.00343947);

      for (int i = 0; i < nullspaceProjector.getNumElements(); i++)
         assertEquals(nullspaceProjectorExpected.get(i), nullspaceProjector.get(i), 1e-7);



      jacobian = new DenseMatrix64F(2, 4);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(0, 3, 7.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 2, 9.0);
      jacobian.set(1, 3, 11.0);


      nullspaceProjector = new DenseMatrix64F(4, 4);
      nullspaceCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      nullspaceProjectorExpected = new DenseMatrix64F(4, 4);

      nullspaceProjectorExpected.set(0, 0, 0.746458);
      nullspaceProjectorExpected.set(0, 1, 0.130345);
      nullspaceProjectorExpected.set(0, 2,-0.381845);
      nullspaceProjectorExpected.set(0, 3,-0.162561);

      nullspaceProjectorExpected.set(1, 0, 0.130345);
      nullspaceProjectorExpected.set(1, 1, 0.708734);
      nullspaceProjectorExpected.set(1, 2, 0.292415);
      nullspaceProjectorExpected.set(1, 3,-0.322225);

      nullspaceProjectorExpected.set(2, 0,-0.381845);
      nullspaceProjectorExpected.set(2, 1, 0.292415);
      nullspaceProjectorExpected.set(2, 2, 0.383735);
      nullspaceProjectorExpected.set(2, 3,-0.0709106);

      nullspaceProjectorExpected.set(3, 0,-0.162561);
      nullspaceProjectorExpected.set(3, 1,-0.322225);
      nullspaceProjectorExpected.set(3, 2,-0.0709106);
      nullspaceProjectorExpected.set(3, 3, 0.161473);

      for (int i = 0; i < nullspaceProjector.getNumElements(); i++)
         assertEquals(nullspaceProjectorExpected.get(i), nullspaceProjector.get(i), 1e-5);




      jacobian = new DenseMatrix64F(3, 4);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(0, 3, 7.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 2, 9.0);
      jacobian.set(1, 3, 11.0);
      jacobian.set(2, 1, 13.0);
      jacobian.set(2, 2, 15.0);
      jacobian.set(2, 3, 17.0);

      nullspaceProjector = new DenseMatrix64F(4, 4);
      nullspaceCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      nullspaceProjectorExpected = new DenseMatrix64F(4, 4);

      nullspaceProjectorExpected.set(0, 0, 0.501058);
      nullspaceProjectorExpected.set(0, 1, 0.423592);
      nullspaceProjectorExpected.set(0, 2,-0.0802936);
      nullspaceProjectorExpected.set(0, 3,-0.253097);

      nullspaceProjectorExpected.set(1, 0, 0.423592);
      nullspaceProjectorExpected.set(1, 1, 0.35831);
      nullspaceProjectorExpected.set(1, 2,-0.0679324);
      nullspaceProjectorExpected.set(1, 3,-0.214036);

      nullspaceProjectorExpected.set(2, 0,-0.0802936);
      nullspaceProjectorExpected.set(2, 1,-0.0679324);
      nullspaceProjectorExpected.set(2, 2, 0.0131833);
      nullspaceProjectorExpected.set(2, 3, 0.0403421);

      nullspaceProjectorExpected.set(3, 0,-0.253097);
      nullspaceProjectorExpected.set(3, 1,-0.214036);
      nullspaceProjectorExpected.set(3, 2, 0.0403421);
      nullspaceProjectorExpected.set(3, 3, 0.128071);

      for (int i = 0; i < nullspaceProjectorExpected.getNumElements(); i++)
         assertEquals(nullspaceProjectorExpected.get(i), nullspaceProjector.get(i), 1e-5);


      jacobian = new DenseMatrix64F(2, 4);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(0, 3, 7.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 2, 9.0);
      jacobian.set(1, 3, 11.0);

      nullspaceProjector = new DenseMatrix64F(4, 4);
      nullspaceCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      nullspaceProjectorExpected = new DenseMatrix64F(4, 4);

      nullspaceProjectorExpected.set(0, 0, 0.746458);
      nullspaceProjectorExpected.set(0, 1, 0.130345);
      nullspaceProjectorExpected.set(0, 2,-0.381845);
      nullspaceProjectorExpected.set(0, 3,-0.162561);

      nullspaceProjectorExpected.set(1, 0, 0.130345);
      nullspaceProjectorExpected.set(1, 1, 0.708734);
      nullspaceProjectorExpected.set(1, 2, 0.292415);
      nullspaceProjectorExpected.set(1, 3,-0.322225);

      nullspaceProjectorExpected.set(2, 0,-0.381845);
      nullspaceProjectorExpected.set(2, 1, 0.292415);
      nullspaceProjectorExpected.set(2, 2, 0.383735);
      nullspaceProjectorExpected.set(2, 3,-0.0709106);

      nullspaceProjectorExpected.set(3, 0,-0.162561);
      nullspaceProjectorExpected.set(3, 1,-0.322225);
      nullspaceProjectorExpected.set(3, 2,-0.0709106);
      nullspaceProjectorExpected.set(3, 3, 0.161473);

      for (int i = 0; i < nullspaceProjectorExpected.getNumElements(); i++)
         assertEquals(nullspaceProjectorExpected.get(i), nullspaceProjector.get(i), 1e-5);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleProjectOntoNullspaceWithDamping()
   {
      DampedNullspaceCalculator nullspaceCalculator = getDampedNullspaceProjectorCalculator();
      nullspaceCalculator.setPseudoInverseAlpha(0.1);

      DenseMatrix64F jacobian = new DenseMatrix64F(2, 2);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 1, 9.0);

      DenseMatrix64F vectorToProject = new DenseMatrix64F(1, 2);
      vectorToProject.set(0, 0, 3.5);
      vectorToProject.set(0, 1, 4.5);

      DenseMatrix64F projectedVector = new DenseMatrix64F(1, 2);

      nullspaceCalculator.projectOntoNullspace(vectorToProject, jacobian, projectedVector);

      assertEquals(0.00124037, projectedVector.get(0, 0), 1e-7);
      assertEquals(-0.00040956, projectedVector.get(0, 1), 1e-7);





      jacobian = new DenseMatrix64F(2, 4);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(0, 3, 7.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 2, 9.0);
      jacobian.set(1, 3, 11.0);

      DenseMatrix64F matrixToProject = new DenseMatrix64F(2, 4);
      matrixToProject.set(0, 0, 3.5);
      matrixToProject.set(0, 1, 4.5);
      matrixToProject.set(0, 2, 5.5);
      matrixToProject.set(0, 3, 6.5);
      matrixToProject.set(1, 0, 7.5);
      matrixToProject.set(1, 1, 8.5);
      matrixToProject.set(1, 2, 9.5);
      matrixToProject.set(1, 3, 10.5);

      projectedVector = new DenseMatrix64F(2, 4);

      nullspaceCalculator.projectOntoNullspace(matrixToProject, jacobian, projectedVector);

      double epsilon = 1e-4;
      assertEquals(0.042359, projectedVector.get(0, 0), epsilon);
      assertEquals(3.15933, projectedVector.get(0, 1), epsilon);
      assertEquals(1.62903, projectedVector.get(0, 2), epsilon);
      assertEquals(-1.35941, projectedVector.get(0, 3), epsilon);

      assertEquals(1.37194, projectedVector.get(1, 0), epsilon);
      assertEquals(6.3964, projectedVector.get(1, 1), epsilon);
      assertEquals(2.52261, projectedVector.get(1, 2), epsilon);
      assertEquals(-2.93631, projectedVector.get(1, 3), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRandomProblemsAgainstTrueDampedLeastSquaresProjection()
   {
      Random random = new Random(12345L);

      DampedNullspaceCalculator nullspaceCalculator = getDampedNullspaceProjectorCalculator();
      double alpha = RandomNumbers.nextDouble(random, 0.01, 2.0);
      nullspaceCalculator.setPseudoInverseAlpha(alpha);
      LinearSolver<DenseMatrix64F> linearSolver = LinearSolverFactory.pseudoInverse(true);

      for (int i = 0; i < 1000; i++)
      {
         int JCols = RandomNumbers.nextInt(random, 1, 100);
         int JRows = RandomNumbers.nextInt(random, 1, 100);

         int ARows = RandomNumbers.nextInt(random, 1, 100);

         double[] JValues = RandomNumbers.nextDoubleArray(random, JRows * JCols, 10.0);
         double[] AValues = RandomNumbers.nextDoubleArray(random, ARows * JCols, 10.0);

         DenseMatrix64F jacobian = new DenseMatrix64F(JRows, JCols, false, JValues);
         DenseMatrix64F jacobianInverse = new DenseMatrix64F(JCols, JRows);
         DenseMatrix64F inverse = new DenseMatrix64F(JRows, JRows);
         DenseMatrix64F matrixToProject = new DenseMatrix64F(ARows, JCols, false, AValues);
         DenseMatrix64F projectedMatrixExpected = new DenseMatrix64F(ARows, JCols);
         DenseMatrix64F projectedMatrix = new DenseMatrix64F(ARows, JCols);

         DenseMatrix64F projectorExpected = new DenseMatrix64F(JCols, JCols);
         DenseMatrix64F projector = new DenseMatrix64F(JCols, JCols);

         nullspaceCalculator.computeNullspaceProjector(jacobian, projector);

         // I - J<sup>+</sup> J
         MatrixTools.setDiagonal(inverse, alpha * alpha);
         CommonOps.multAddTransB(jacobian, jacobian, inverse);
         linearSolver.setA(inverse);
         linearSolver.invert(jacobianInverse);

         DenseMatrix64F tempMatrix = new DenseMatrix64F(JCols, JRows);
         CommonOps.multTransA(jacobian, jacobianInverse, tempMatrix);
         CommonOps.mult(tempMatrix, jacobian, projectorExpected);
         CommonOps.scale(-1.0, projectorExpected);
         MatrixTools.addDiagonal(projectorExpected, 1.0);

         for (int j = 0; j < projectorExpected.getNumRows(); j++)
         {
            for (int k = 0; k < projectorExpected.getNumCols(); k++)
            {
               assertEquals("Iteration " + i + " failed on index " + " (" + j + ", " + k + ").", projectorExpected.get(j, k), projector.get(j, k), 1e-7);
            }
         }




         // compute the projection
         CommonOps.mult(matrixToProject, projectorExpected, projectedMatrixExpected);
         nullspaceCalculator.projectOntoNullspace(matrixToProject, jacobian, projectedMatrix);
         nullspaceCalculator.projectOntoNullspace(matrixToProject, jacobian);

         for (int j = 0; j < projectedMatrixExpected.getNumRows(); j++)
         {
            for (int k = 0; k < projectedMatrixExpected.getNumCols(); k++)
            {
               assertEquals("Iteration " + i + " failed on index " + " (" + j + ", " + k + ").", projectedMatrixExpected.get(j, k), projectedMatrix.get(j, k), 1e-7);
               assertEquals("Iteration " + i + " failed on index " + " (" + j + ", " + k + ").", projectedMatrixExpected.get(j, k), matrixToProject.get(j, k), 1e-7);
            }
         }
      }

   }
}
