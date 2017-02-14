package us.ihmc.convexOptimization.quadraticProgram;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.random.RandomTools;

public class SimpleInefficientEqualityConstrainedQPSolverTest
{
   private static final boolean VERBOSE = false;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleCases()
   {
      SimpleInefficientEqualityConstrainedQPSolver solver = new SimpleInefficientEqualityConstrainedQPSolver();

      // Minimize x^T * x
      double[][] costQuadraticMatrix = new double[][] { { 2.0 } };
      double[] costLinearVector = new double[] { 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[] solution = new double[1];
      double[] lagrangeMultipliers = new double[0];
      solver.solve(solution, lagrangeMultipliers);
      assertEquals(1, solution.length);
      assertEquals(0.0, solution[0], 1e-7);

      // Minimize (x-5) * (x-5) = x^2 - 10x + 25
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { -10.0 };
      quadraticCostScalar = 25.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      solution = new double[1];
      lagrangeMultipliers = new double[0];
      solver.solve(solution, lagrangeMultipliers);

      assertEquals(1, solution.length);
      assertEquals(5.0, solution[0], 1e-7);
      DenseMatrix64F solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(0.0, objectiveCost, 1e-7);

      // Minimize (x-5) * (x-5) + (y-3) * (y-3) = 1/2 * (2x^2 + 2y^2) - 10x -6y + 34
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { -10.0, -6.0 };
      quadraticCostScalar = 34.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      solution = new double[2];
      lagrangeMultipliers = new double[0];
      solver.solve(solution, lagrangeMultipliers);

      assertEquals(2, solution.length);
      assertEquals(5.0, solution[0], 1e-7);
      assertEquals(3.0, solution[1], 1e-7);
      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(0.0, objectiveCost, 1e-7);

      // Minimize x^2 + y^2 subject to x + y = 1.0
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 1.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      solution = new double[2];
      lagrangeMultipliers = new double[1];
      solver.solve(solution, lagrangeMultipliers);

      assertEquals(2, solution.length);
      assertEquals(0.5, solution[0], 1e-7);
      assertEquals(0.5, solution[1], 1e-7);
      assertEquals(-1.0, lagrangeMultipliers[0], 1e-7); // Lagrange multiplier is -1.0;
      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(0.5, objectiveCost, 1e-7);

      // Minimize x^2 + y^2 subject to x + y = 2.0, 3x - 3y = 0.0
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 }, { 3.0, -3.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0, 0.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      solution = new double[2];
      lagrangeMultipliers = new double[2];
      solver.solve(solution, lagrangeMultipliers);

      assertEquals(2, solution.length);
      assertEquals(1.0, solution[0], 1e-7);
      assertEquals(1.0, solution[1], 1e-7);
      assertEquals(-2.0, lagrangeMultipliers[0], 1e-7); // Lagrange multiplier
      assertEquals(0.0, lagrangeMultipliers[1], 1e-7); // Lagrange multiplier
      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(2.0, objectiveCost, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testLargeRandomProblemWithNoEqualityConstraints()
   {
      Random random = new Random(1776L);

      SimpleInefficientEqualityConstrainedQPSolver solver = new SimpleInefficientEqualityConstrainedQPSolver();

      int numberOfTests = 100;

      for (int testNumber = 0; testNumber < numberOfTests; testNumber++)
      {
         solver.clear();
         int numberOfVariables = 100;

         DenseMatrix64F costQuadraticMatrix = RandomTools.generateRandomMatrix(random, numberOfVariables, numberOfVariables);
         DenseMatrix64F identity = CommonOps.identity(numberOfVariables, numberOfVariables); // Add n*I to make sure it is positive definite...
         CommonOps.scale(numberOfVariables, identity);
         CommonOps.addEquals(costQuadraticMatrix, identity);

         DenseMatrix64F costLinearVector = RandomTools.generateRandomMatrix(random, numberOfVariables, 1);
         double quadraticCostScalar = RandomTools.generateRandomDouble(random, 30.0);

         solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

         double[] solution = new double[numberOfVariables];
         double[] lagrangeMultipliers = new double[0];
         solver.solve(solution, lagrangeMultipliers);

         assertEquals(numberOfVariables, solution.length);

         DenseMatrix64F solutionMatrix = new DenseMatrix64F(numberOfVariables, 1);
         solutionMatrix.setData(solution);
         double objectiveCost = solver.getObjectiveCost(solutionMatrix);

         double[] solutionWithSmallPerturbation = new double[numberOfVariables];
         for (int i = 0; i < numberOfVariables; i++)
         {
            solutionWithSmallPerturbation[i] = solution[i] + RandomTools.generateRandomDouble(random, 1e-7);
         }

         solutionMatrix = new DenseMatrix64F(numberOfVariables, 1);
         solutionMatrix.setData(solutionWithSmallPerturbation);
         double objectiveCostWithSmallPerturbation = solver.getObjectiveCost(solutionMatrix);

         assertTrue("objectiveCostWithSmallPerturbation = " + objectiveCostWithSmallPerturbation + ", objectiveCost = " + objectiveCost, objectiveCostWithSmallPerturbation > objectiveCost);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testLargeRandomProblemWithEqualityConstraints()
   {
      Random random = new Random(1776L);

      SimpleInefficientEqualityConstrainedQPSolver solver = new SimpleInefficientEqualityConstrainedQPSolver();

      int numberOfTests = 100;

      long startTimeMillis = System.currentTimeMillis();

      for (int testNumber = 0; testNumber < numberOfTests; testNumber++)
      {
         solver.clear();
         int numberOfVariables = 80;
         int numberOfEqualityConstraints = 16;

         DenseMatrix64F costQuadraticMatrix = RandomTools.generateRandomMatrix(random, numberOfVariables, numberOfVariables);
         DenseMatrix64F identity = CommonOps.identity(numberOfVariables, numberOfVariables); // Add n*I to make sure it is positive definite...
         CommonOps.scale(numberOfVariables, identity);
         CommonOps.addEquals(costQuadraticMatrix, identity);

         DenseMatrix64F costLinearVector = RandomTools.generateRandomMatrix(random, numberOfVariables, 1);
         double quadraticCostScalar = RandomTools.generateRandomDouble(random, 30.0);

         solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

         DenseMatrix64F linearEqualityConstraintsAMatrix = RandomTools.generateRandomMatrix(random, numberOfEqualityConstraints, numberOfVariables);
         DenseMatrix64F linearEqualityConstraintsBVector = RandomTools.generateRandomMatrix(random, numberOfEqualityConstraints, 1);
         solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

         double[] solution = new double[numberOfVariables];
         double[] lagrangeMultipliers = new double[numberOfEqualityConstraints];
         solver.solve(solution, lagrangeMultipliers);

         assertEquals(numberOfVariables, solution.length);
         assertEquals(numberOfEqualityConstraints, lagrangeMultipliers.length);

         DenseMatrix64F solutionMatrix = new DenseMatrix64F(numberOfVariables, 1);
         solutionMatrix.setData(solution);
         double objectiveCost = solver.getObjectiveCost(solutionMatrix);

         // Verify equality constraints hold:
         verifyEqualityConstraintsHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);

         // Verify objective is minimized by comparing to small perturbation:
         double[] solutionWithSmallPerturbation = new double[numberOfVariables];
         for (int i = 0; i < numberOfVariables; i++)
         {
            solutionWithSmallPerturbation[i] = solution[i] + RandomTools.generateRandomDouble(random, 1e-4);
         }

         solutionMatrix = new DenseMatrix64F(numberOfVariables, 1);
         solutionMatrix.setData(solutionWithSmallPerturbation);

         verifyEqualityConstraintsDoNotHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);
         DenseMatrix64F solutionMatrixProjectedOntoEqualityConstraints = projectOntoEqualityConstraints(solutionMatrix, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);
         verifyEqualityConstraintsHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrixProjectedOntoEqualityConstraints);

         double objectiveCostWithSmallPerturbation = solver.getObjectiveCost(solutionMatrixProjectedOntoEqualityConstraints);

         assertTrue("objectiveCostWithSmallPerturbation = " + objectiveCostWithSmallPerturbation + ", objectiveCost = " + objectiveCost, objectiveCostWithSmallPerturbation > objectiveCost);
      }

      long endTimeMillis = System.currentTimeMillis();

      double timePerTest = ((double) (endTimeMillis - startTimeMillis)) * 0.001 / ((double) numberOfTests);

      if (VERBOSE)
      {
         System.out.println("Time per test is " + timePerTest);
      }

   }

   private void verifyEqualityConstraintsHold(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      double maxAbsoluteError = getMaxEqualityConstraintError(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);
      assertEquals(0.0, maxAbsoluteError, 1e-5);
   }

   private void verifyEqualityConstraintsDoNotHold(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      double maxAbsoluteError = getMaxEqualityConstraintError(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);
      assertTrue(maxAbsoluteError > 1e-5);
   }

   private double getMaxEqualityConstraintError(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      DenseMatrix64F checkMatrix = new DenseMatrix64F(numberOfEqualityConstraints, 1);
      CommonOps.mult(linearEqualityConstraintsAMatrix, solutionMatrix, checkMatrix);
      CommonOps.subtractEquals(checkMatrix, linearEqualityConstraintsBVector);

      return getMaxAbsoluteDataEntry(checkMatrix);
   }

   private DenseMatrix64F projectOntoEqualityConstraints(DenseMatrix64F solutionMatrix, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector)
   {
      int numberOfVariables = solutionMatrix.getNumRows();
      if (linearEqualityConstraintsAMatrix.getNumCols() != numberOfVariables)
         throw new RuntimeException();

      int numberOfConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      if (linearEqualityConstraintsBVector.getNumRows() != numberOfConstraints)
         throw new RuntimeException();

      DenseMatrix64F AZMinusB = new DenseMatrix64F(numberOfConstraints, 1);
      CommonOps.mult(linearEqualityConstraintsAMatrix, solutionMatrix, AZMinusB);
      CommonOps.subtractEquals(AZMinusB, linearEqualityConstraintsBVector);

      DenseMatrix64F AATransposeInverse = new DenseMatrix64F(numberOfConstraints, numberOfConstraints);
      DenseMatrix64F linearEqualityConstraintsAMatrixTranspose = new DenseMatrix64F(linearEqualityConstraintsAMatrix);
      CommonOps.transpose(linearEqualityConstraintsAMatrixTranspose);

      CommonOps.mult(linearEqualityConstraintsAMatrix, linearEqualityConstraintsAMatrixTranspose, AATransposeInverse);
      CommonOps.invert(AATransposeInverse);

      DenseMatrix64F ATransposeAATransposeInverse = new DenseMatrix64F(numberOfVariables, numberOfConstraints);
      CommonOps.mult(linearEqualityConstraintsAMatrixTranspose, AATransposeInverse, ATransposeAATransposeInverse);

      DenseMatrix64F vectorToSubtract = new DenseMatrix64F(numberOfVariables, 1);
      CommonOps.mult(ATransposeAATransposeInverse, AZMinusB, vectorToSubtract);

      DenseMatrix64F projectedSolutionMatrix = new DenseMatrix64F(solutionMatrix);
      CommonOps.subtractEquals(projectedSolutionMatrix, vectorToSubtract);

      return projectedSolutionMatrix;
   }

   private double getMaxAbsoluteDataEntry(DenseMatrix64F matrix)
   {
      int numberOfRows = matrix.getNumRows();
      int numberOfColumns = matrix.getNumCols();

      double max = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            double absoluteValue = Math.abs(matrix.get(i, j));
            if (absoluteValue > max)
            {
               max = absoluteValue;
            }
         }
      }

      return max;
   }

}
