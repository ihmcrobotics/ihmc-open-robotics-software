package us.ihmc.robotics.linearAlgebra;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTools;

public abstract class NullspaceCalculatorTest
{
   public abstract NullspaceCalculator getNullspaceProjectorCalculator();

   @Test
   public void testSimpleNullspaceProjector()
   {
      NullspaceCalculator nullspaceCalculator = getNullspaceProjectorCalculator();

      DMatrixRMaj jacobian = new DMatrixRMaj(2, 2);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 1, 9.0);

      DMatrixRMaj nullspaceProjector = new DMatrixRMaj(2, 2);
      nullspaceCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      for (int i = 0; i < nullspaceProjector.getNumElements(); i++)
         assertEquals(nullspaceProjector.get(i), 0.0, 1e-7);


      jacobian = new DMatrixRMaj(2, 4);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(0, 3, 7.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 2, 9.0);
      jacobian.set(1, 3, 11.0);

      nullspaceProjector = new DMatrixRMaj(4, 4);
      nullspaceCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      DMatrixRMaj nullspaceProjectorExpected = new DMatrixRMaj(4, 4);

      nullspaceProjectorExpected.set(0, 0, 0.746421);
      nullspaceProjectorExpected.set(0, 1, 0.130401);
      nullspaceProjectorExpected.set(0, 2,-0.381917);
      nullspaceProjectorExpected.set(0, 3,-0.162518);

      nullspaceProjectorExpected.set(1, 0, 0.130401);
      nullspaceProjectorExpected.set(1, 1, 0.708629);
      nullspaceProjectorExpected.set(1, 2, 0.292532);
      nullspaceProjectorExpected.set(1, 3,-0.322327);

      nullspaceProjectorExpected.set(2, 0,-0.381917);
      nullspaceProjectorExpected.set(2, 1, 0.292532);
      nullspaceProjectorExpected.set(2, 2, 0.383593);
      nullspaceProjectorExpected.set(2, 3,-0.0708113);

      nullspaceProjectorExpected.set(3, 0,-0.162518);
      nullspaceProjectorExpected.set(3, 1,-0.322327);
      nullspaceProjectorExpected.set(3, 2,-0.0708113);
      nullspaceProjectorExpected.set(3, 3, 0.161357);

      for (int i = 0; i < nullspaceProjectorExpected.getNumElements(); i++)
         assertEquals(nullspaceProjectorExpected.get(i), nullspaceProjector.get(i), 1e-5);


      jacobian = new DMatrixRMaj(5, 5);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 2, 3.0);
      jacobian.set(0, 3, 5.0);
      jacobian.set(0, 4, 7.0);

      jacobian.set(1, 1, 9.0);
      jacobian.set(1, 2, 11.0);
      jacobian.set(1, 3, 13.0);
      jacobian.set(1, 4, 15.0);

      jacobian.set(2, 0, 17.0);
      jacobian.set(2, 1, 19.0);
      jacobian.set(2, 2, 21.0);
      jacobian.set(2, 3, 23.0);

      jacobian.set(3, 0, 25.0);
      jacobian.set(3, 1, 27.0);
      jacobian.set(3, 2, 29.0);
      jacobian.set(3, 3, 31.0);
      jacobian.set(3, 4, 33.0);

      jacobian.set(4, 0, 35.0);
      jacobian.set(4, 1, 37.0);
      jacobian.set(4, 2, 39.0);
      jacobian.set(4, 4, 41.0);

      nullspaceProjector = new DMatrixRMaj(5, 5);
      nullspaceCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      nullspaceProjectorExpected = new DMatrixRMaj(5, 5);

      for (int i = 0; i < nullspaceProjectorExpected.getNumElements(); i++)
         assertEquals(nullspaceProjectorExpected.get(i), nullspaceProjector.get(i), 1e-5);
   }

   @Test
   public void testSimpleProjectOntoNullspace()
   {
      NullspaceCalculator nullspaceCalculator = getNullspaceProjectorCalculator();

      DMatrixRMaj jacobian = new DMatrixRMaj(2, 2);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 1, 9.0);

      DMatrixRMaj vectorToProject = new DMatrixRMaj(1, 2);
      vectorToProject.set(0, 0, 3.5);
      vectorToProject.set(0, 1, 4.5);

      DMatrixRMaj projectedVector = new DMatrixRMaj(1, 2);

      nullspaceCalculator.projectOntoNullspace(vectorToProject, jacobian, projectedVector);

      assertEquals(0.0, projectedVector.get(0, 0), 1e-7);
      assertEquals(0.0, projectedVector.get(0, 1), 1e-7);


      jacobian = new DMatrixRMaj(2, 4);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(0, 3, 7.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 2, 9.0);
      jacobian.set(1, 3, 11.0);

      DMatrixRMaj matrixToProject = new DMatrixRMaj(2, 4);
      matrixToProject.set(0, 0, 3.5);
      matrixToProject.set(0, 1, 4.5);
      matrixToProject.set(0, 2, 5.5);
      matrixToProject.set(0, 3, 6.5);
      matrixToProject.set(1, 0, 7.5);
      matrixToProject.set(1, 1, 8.5);
      matrixToProject.set(1, 2, 9.5);
      matrixToProject.set(1, 3, 10.5);

      projectedVector = new DMatrixRMaj(2, 4);

      nullspaceCalculator.projectOntoNullspace(matrixToProject, jacobian, projectedVector);

      double epsilon = 1e-5;
      assertEquals(0.0423707, projectedVector.get(0, 0), epsilon);
      assertEquals(3.15904, projectedVector.get(0, 1), epsilon);
      assertEquals(1.62918, projectedVector.get(0, 2), epsilon);
      assertEquals(-1.35993, projectedVector.get(0, 3), epsilon);

      assertEquals(1.37192, projectedVector.get(1, 0), epsilon);
      assertEquals(6.39598, projectedVector.get(1, 1), epsilon);
      assertEquals(2.52277, projectedVector.get(1, 2), epsilon);
      assertEquals(-2.93712, projectedVector.get(1, 3), epsilon);

      jacobian = new DMatrixRMaj(5, 5);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 2, 3.0);
      jacobian.set(0, 3, 5.0);
      jacobian.set(0, 4, 7.0);

      jacobian.set(1, 1, 9.0);
      jacobian.set(1, 2, 11.0);
      jacobian.set(1, 3, 13.0);
      jacobian.set(1, 4, 15.0);

      jacobian.set(2, 0, 17.0);
      jacobian.set(2, 1, 19.0);
      jacobian.set(2, 2, 21.0);
      jacobian.set(2, 3, 23.0);

      jacobian.set(3, 0, 25.0);
      jacobian.set(3, 1, 27.0);
      jacobian.set(3, 2, 29.0);
      jacobian.set(3, 3, 31.0);
      jacobian.set(3, 4, 33.0);

      jacobian.set(4, 0, 35.0);
      jacobian.set(4, 1, 37.0);
      jacobian.set(4, 2, 39.0);
      jacobian.set(4, 4, 41.0);

      matrixToProject = new DMatrixRMaj(2, 5);
      matrixToProject.set(0, 0, 3.5);
      matrixToProject.set(0, 1, 4.5);
      matrixToProject.set(0, 2, 5.5);
      matrixToProject.set(0, 3, 6.5);
      matrixToProject.set(0, 4, 7.5);
      matrixToProject.set(1, 0, 8.5);
      matrixToProject.set(1, 1, 9.5);
      matrixToProject.set(1, 2, 10.5);
      matrixToProject.set(1, 3, 11.5);
      matrixToProject.set(1, 4, 13.5);

      projectedVector.reshape(2, 5);
      nullspaceCalculator.projectOntoNullspace(matrixToProject, jacobian, projectedVector);

      epsilon = 1e-5;
      assertEquals(0.0, projectedVector.get(0, 0), epsilon);
      assertEquals(0.0, projectedVector.get(0, 1), epsilon);
      assertEquals(0.0, projectedVector.get(0, 2), epsilon);
      assertEquals(0.0, projectedVector.get(0, 3), epsilon);
      assertEquals(0.0, projectedVector.get(0, 4), epsilon);

      assertEquals(0.0, projectedVector.get(1, 0), epsilon);
      assertEquals(0.0, projectedVector.get(1, 1), epsilon);
      assertEquals(0.0, projectedVector.get(1, 2), epsilon);
      assertEquals(0.0, projectedVector.get(1, 3), epsilon);
      assertEquals(0.0, projectedVector.get(1, 4), epsilon);
   }

   @Test
   public void testRandomProblemsAgainstTrueLeastSquaresProjection()
   {
      Random random = new Random(12345L);

      NullspaceCalculator nullspaceCalculator = getNullspaceProjectorCalculator();
      LinearSolverDense<DMatrixRMaj> linearSolver = LinearSolverFactory_DDRM.pseudoInverse(true);

      for (int i = 0; i < 1000; i++)
      {
         int JCols = RandomNumbers.nextInt(random, 1, 100);
         int JRows = RandomNumbers.nextInt(random, 1, 100);

         int ARows = RandomNumbers.nextInt(random, 1, 100);

         double[] JValues = RandomNumbers.nextDoubleArray(random, JRows * JCols, 10.0);
         double[] AValues = RandomNumbers.nextDoubleArray(random, ARows * JCols, 10.0);

         DMatrixRMaj jacobian = new DMatrixRMaj(JRows, JCols, false, JValues);
         DMatrixRMaj jacobianInverse = new DMatrixRMaj(JCols, JRows);
         DMatrixRMaj matrixToProject = new DMatrixRMaj(ARows, JCols, false, AValues);
         DMatrixRMaj projectedMatrixExpected = new DMatrixRMaj(ARows, JCols);
         DMatrixRMaj projectedMatrix = new DMatrixRMaj(ARows, JCols);

         DMatrixRMaj projectorExpected = new DMatrixRMaj(JCols, JCols);
         DMatrixRMaj projector = new DMatrixRMaj(JCols, JCols);

         nullspaceCalculator.computeNullspaceProjector(jacobian, projector);

         /** I - J^-1 * J */
         linearSolver.setA(jacobian);
         linearSolver.invert(jacobianInverse);

         CommonOps_DDRM.mult(jacobianInverse, jacobian, projectorExpected);
         CommonOps_DDRM.scale(-1.0, projectorExpected);
         MatrixTools.addDiagonal(projectorExpected, 1.0);

         for (int j = 0; j < projectorExpected.getNumRows(); j++)
         {
            for (int k = 0; k < projectorExpected.getNumCols(); k++)
            {
               assertEquals("Iteration " + i + " failed on index " + " (" + j + ", " + k + ").", projectorExpected.get(j, k), projector.get(j, k), 1e-7);
            }
         }


         // compute the projection
         CommonOps_DDRM.mult(matrixToProject, projectorExpected, projectedMatrixExpected);
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
