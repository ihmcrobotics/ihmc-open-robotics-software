package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class JavaQuadProgSolverTest
{
   private static final boolean VERBOSE = true;
   private static final double epsilon = 1e-4;

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testTimingAgainstStandardQuadProg() throws NoConvergenceException
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      ExecutionTimer javaSolverTimer = new ExecutionTimer("javaSolverTimer", registry);
      ExecutionTimer wrapperSolverTimer = new ExecutionTimer("wrapperSolverTimer", registry);

      int numberOfInequalityConstraints = 1;
      int numberOfEqualityConstraints = 2;
      int numberOfVariables = 3;

      DenseMatrix64F Q = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 1, 0, 1, 2, 1, 3, 7);
      DenseMatrix64F f = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0, 9);
      DenseMatrix64F Aeq = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1, 1, 2, 3, 4);
      DenseMatrix64F beq = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 0, 7);
      DenseMatrix64F Ain = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1, 3);
      DenseMatrix64F bin = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      for (int repeat = 0; repeat < 10000; repeat++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1, 3);
         DenseMatrix64F xWrapper = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1, 3);

         JavaQuadProgSolver javaSolver = new JavaQuadProgSolver();
         javaSolverTimer.startMeasurement();

         javaSolver.clear();
         javaSolver.setQuadraticCostFunction(Q, f, 0.0);
         javaSolver.setLinearInequalityConstraints(Ain, bin);
         javaSolver.setLinearEqualityConstraints(Aeq, beq);
         javaSolver.solve(x);

         javaSolverTimer.stopMeasurement();

         QuadProgSolver solver = new QuadProgSolver();
         wrapperSolverTimer.startMeasurement();
         solver.solve(Q, f, Aeq, beq, Ain, bin, xWrapper, false);
         wrapperSolverTimer.stopMeasurement();

         DenseMatrix64F bEqualityVerify = new DenseMatrix64F(numberOfEqualityConstraints, 1);
         CommonOps.mult(Aeq, x, bEqualityVerify);

         // Verify Ax=b Equality constraints hold:
         JUnitTools.assertMatrixEquals(bEqualityVerify, beq, epsilon);

         // Verify Ax<b Inequality constraints hold:
         DenseMatrix64F bInequalityVerify = new DenseMatrix64F(numberOfInequalityConstraints, 1);
         CommonOps.mult(Ain, x, bInequalityVerify);

         for (int j=0; j<bInequalityVerify.getNumRows(); j++)
         {
            Assert.assertTrue(bInequalityVerify.get(j, 0) < beq.get(j, 0));
         }

         // Verify solution is as expected
         Assert.assertArrayEquals("repeat = " + repeat, x.getData(), xWrapper.getData(), 1e-10);
      }

      PrintTools.info("Wrapper solve time : " + wrapperSolverTimer.getAverageTime());
      PrintTools.info("Java solve time : " + javaSolverTimer.getAverageTime());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testAgainstStandardQuadProg() throws NoConvergenceException
   {
      int numberOfInequalityConstraints = 1;
      int numberOfEqualityConstraints = 1;
      int numberOfVariables = 2;

      DenseMatrix64F Q = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 0, 1);
      DenseMatrix64F f = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0);
      DenseMatrix64F Aeq = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1);
      DenseMatrix64F beq = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 0);
      DenseMatrix64F Ain = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1);
      DenseMatrix64F bin = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      JavaQuadProgSolver javaSolver = new JavaQuadProgSolver();
      QuadProgSolver solver = new QuadProgSolver();

      for (int repeat = 0; repeat < 10000; repeat++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1);
         javaSolver.clear();
         javaSolver.setQuadraticCostFunction(Q, f, 0.0);
         javaSolver.setLinearInequalityConstraints(Ain, bin);
         javaSolver.setLinearEqualityConstraints(Aeq, beq);
         javaSolver.solve(x);
         Assert.assertArrayEquals(x.getData(), new double[] { -0.5, 0.5 }, 1e-10);
      }

      numberOfInequalityConstraints = 1;
      numberOfEqualityConstraints = 2;
      numberOfVariables = 3;

      Q = new DenseMatrix64F(numberOfVariables, numberOfVariables, true, 1, 0, 1, 0, 1, 2, 1, 3, 7);
      f = new DenseMatrix64F(numberOfVariables, 1, true, 1, 0, 9);
      Aeq = new DenseMatrix64F(numberOfEqualityConstraints, numberOfVariables, true, 1, 1, 1, 2, 3, 4);
      beq = new DenseMatrix64F(numberOfEqualityConstraints, 1, true, 0, 7);
      Ain = new DenseMatrix64F(numberOfInequalityConstraints, numberOfVariables, true, 2, 1, 3);
      bin = new DenseMatrix64F(numberOfInequalityConstraints, 1, true, 0);

      for (int repeat = 0; repeat < 10000; repeat++)
      {
         DenseMatrix64F x = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1, 3);
         DenseMatrix64F xWrapper = new DenseMatrix64F(numberOfVariables, 1, true, -1, 1, 3);

         javaSolver.clear();
         javaSolver.setQuadraticCostFunction(Q, f, 0.0);
         javaSolver.setLinearEqualityConstraints(Aeq, beq);
         javaSolver.setLinearInequalityConstraints(Ain, bin);
         javaSolver.solve(x);
         solver.solve(Q, f, Aeq, beq, Ain, bin, xWrapper, false);

         DenseMatrix64F bEqualityVerify = new DenseMatrix64F(numberOfEqualityConstraints, 1);
         CommonOps.mult(Aeq, x, bEqualityVerify);

         // Verify Ax=b Equality constraints hold:
         JUnitTools.assertMatrixEquals(bEqualityVerify, beq, epsilon);

         // Verify Ax<b Inequality constraints hold:
         DenseMatrix64F bInequalityVerify = new DenseMatrix64F(numberOfInequalityConstraints, 1);
         CommonOps.mult(Ain, x, bInequalityVerify);

         for (int j=0; j<bInequalityVerify.getNumRows(); j++)
         {
            Assert.assertTrue(bInequalityVerify.get(j, 0) < beq.get(j, 0));
         }

         // Verify solution is as expected
         Assert.assertArrayEquals("repeat = " + repeat, x.getData(), xWrapper.getData(), 1e-10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testSolveProblemWithParallelConstraints() throws NoConvergenceException
   {
      // our simple active set solver can not solve this:
      // test problem: x <= -1 and x <= -2
      DenseMatrix64F Q = new DenseMatrix64F(1, 1);
      DenseMatrix64F Ain = new DenseMatrix64F(2, 1);
      DenseMatrix64F bin = new DenseMatrix64F(2, 1);
      DenseMatrix64F x = new DenseMatrix64F(1, 1);

      Q.set(0, 0, 1.0);
      Ain.set(0, 0, 1.0);
      Ain.set(1, 0, 1.0);
      bin.set(0, -1.0);
      bin.set(0, -2.0);

      DenseMatrix64F f = new DenseMatrix64F(1, 1);
      DenseMatrix64F Aeq = new DenseMatrix64F(0, 1);
      DenseMatrix64F beq = new DenseMatrix64F(0, 1);

      JavaQuadProgSolver solver = new JavaQuadProgSolver();

      solver.clear();
      solver.setQuadraticCostFunction(Q, f, 0.0);
      solver.setLinearEqualityConstraints(Aeq, beq);
      solver.setLinearInequalityConstraints(Ain, bin);
      solver.solve(x);

      PrintTools.info("Attempting to solve problem with: " + solver.getClass().getSimpleName());
      solver.clear();
      solver.setQuadraticCostFunction(Q, f, 0.0);
      solver.setLinearEqualityConstraints(Aeq, beq);
      solver.setLinearInequalityConstraints(Ain, bin);
      solver.solve(x);


      boolean correct = MathTools.epsilonEquals(-2.0, x.get(0), 10E-10);
      if (!correct)
      {
         PrintTools.info("Failed. Java Result was " + x.get(0) + ", expected -2.0");
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleCasesWithNoInequalityConstraints() throws NoConvergenceException
   {
      JavaQuadProgSolver solver = new JavaQuadProgSolver();

      // Minimize x^T * x
      double[][] costQuadraticMatrix = new double[][] { { 2.0 } };
      double[] costLinearVector = new double[] { 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[] solution = new double[1];
      solver.solve(solution);
      int numberOfIterations = solver.solve(solution); // Make sure ok to solve twice in a row without changing stuff.
      assertEquals(0, numberOfIterations);
      assertEquals(1, solution.length);
      assertEquals(0.0, solution[0], epsilon);

      // Minimize (x-5) * (x-5) = x^2 - 10x + 25
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { -10.0 };
      quadraticCostScalar = 25.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      solution = new double[1];
      numberOfIterations = solver.solve(solution);
      assertEquals(0, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(5.0, solution[0], epsilon);
      DenseMatrix64F solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(0.0, objectiveCost, epsilon);

      // Minimize (x-5) * (x-5) + (y-3) * (y-3) = 1/2 * (2x^2 + 2y^2) - 10x -6y + 34
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { -10.0, -6.0 };
      quadraticCostScalar = 34.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      solution = new double[2];
      numberOfIterations = solver.solve(solution);
      assertEquals(0, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(5.0, solution[0], epsilon);
      assertEquals(3.0, solution[1], epsilon);
      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(0.0, objectiveCost, epsilon);

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
      numberOfIterations = solver.solve(solution);
      assertEquals(0, numberOfIterations);

      assertEquals(0.5, solution[0], epsilon);
      assertEquals(0.5, solution[1], epsilon);
      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(0.5, objectiveCost, epsilon);

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
      numberOfIterations = solver.solve(solution);
      assertEquals(0, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(1.0, solution[0], epsilon);
      assertEquals(1.0, solution[1], epsilon);
      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(2.0, objectiveCost, epsilon);
   }




   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleCasesWithInequalityConstraints() throws NoConvergenceException
   {
      JavaQuadProgSolver solver = new JavaQuadProgSolver();

      // Minimize x^T * x subject to x <= 1
      double[][] costQuadraticMatrix = new double[][] { { 2.0 } };
      double[] costLinearVector = new double[] { 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 1.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { 1.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      double[] solution = new double[1];

      int numberOfIterations = solver.solve(solution);
      assertEquals(0, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(0.0, solution[0], 1e-7);

      // Minimize x^T * x subject to x >= 1 (-x <= -1)
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearInequalityConstraintsCMatrix = new double[][] { { -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -1.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solution = new double[1];
      numberOfIterations = solver.solve(solution);
      assertEquals(1, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(1.0, solution[0], 1e-7);

      // Minimize (x-5) * (x-5) = x^2 - 10x + 25 subject to x <= 3.0
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { -10.0 };
      quadraticCostScalar = 25.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0 } };
      linearInqualityConstraintsDVector = new double[] { 3.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solution = new double[1];
      numberOfIterations = solver.solve(solution);
      assertEquals(1, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(3.0, solution[0], epsilon);

      DenseMatrix64F solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(4.0, objectiveCost, epsilon);

      // Minimize (x-5) * (x-5) + (y-3) * (y-3) = 1/2 * (2x^2 + 2y^2) - 10x -6y + 34 subject to x <= 7 y <= 1
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { -10.0, -6.0 };
      quadraticCostScalar = 34.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 0.0 }, { 0.0, 1.0 } };
      linearInqualityConstraintsDVector = new double[] { 7.0, 1.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solution = new double[2];
      numberOfIterations = solver.solve(solution);
      assertEquals(1, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(5.0, solution[0], epsilon);
      assertEquals(1.0, solution[1], epsilon);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(4.0, objectiveCost, epsilon);

      // Minimize x^2 + y^2 subject to x + y = 1.0, x <= y - 1 (x - y <= -1.0)
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 1.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -1.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solution = new double[2];
      numberOfIterations = solver.solve(solution);
      assertEquals(1, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(0.0, solution[0], epsilon);
      assertEquals(1.0, solution[1], epsilon);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(1.0, objectiveCost, epsilon);

      // Minimize x^2 + y^2 subject to x + y = 2.0, 3x - 3y = 0.0, x <= 2, x <= 10, y <= 3
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 }, { 3.0, -3.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0, 0.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 0.0 }, { 1.0, 0.0 }, { 0.0, 1.0 } };
      linearInqualityConstraintsDVector = new double[] { 2.0, 10.0, 3.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solution = new double[2];
      numberOfIterations = solver.solve(solution);
      assertEquals(0, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(1.0, solution[0], epsilon);
      assertEquals(1.0, solution[1], epsilon);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(2.0, objectiveCost, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000000)
   public void testSimpleCasesWithBoundsConstraints() throws NoConvergenceException
   {
      JavaQuadProgSolver solver = new JavaQuadProgSolver();

      // Minimize x^T * x
      double[][] costQuadraticMatrix = new double[][] { { 2.0 } };
      double[] costLinearVector = new double[] { 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[] variableLowerBounds = new double[] { Double.NEGATIVE_INFINITY };
      double[] variableUpperBounds = new double[] { Double.POSITIVE_INFINITY };
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      double[] solution = new double[1];

      solver.solve(solution);
      int numberOfIterations = solver.solve(solution);
      assertEquals(0, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(0.0, solution[0], epsilon);

      // Minimize x^T * x subject to x >= 1
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      variableLowerBounds = new double[] { 1.0 };
      variableUpperBounds = new double[] { Double.POSITIVE_INFINITY };
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      solution = new double[1];
      numberOfIterations = solver.solve(solution);
      assertEquals(1, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(1.0, solution[0], epsilon);

      // Minimize x^T * x subject to x <= -1
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      variableLowerBounds = new double[] { Double.NEGATIVE_INFINITY };
      variableUpperBounds = new double[] { -1.0 };
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      solution = new double[1];
      numberOfIterations = solver.solve(solution);
      assertEquals(1, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(-1.0, solution[0], epsilon);

      // Minimize x^T * x subject to 1 + 1e-12 <= x <= 1 - 1e-12 (Should give valid solution given a little epsilon to allow for roundoff)
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      variableLowerBounds = new double[] { 1.0 - 1e-12};
      variableUpperBounds = new double[] { 1.0 + 1e-12};
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      solution = new double[1];
      numberOfIterations = solver.solve(solution);
      assertEquals(1, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(1.0, solution[0], epsilon);

      // Minimize x^T * x subject to -1 + 1e-12 <= x <= -1 - 1e-12 (Should give valid solution given a little epsilon to allow for roundoff)
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      variableLowerBounds = new double[] { -1.0 - 1e-12};
      variableUpperBounds = new double[] { -1.0 + 1e-12};
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      solution = new double[1];
      numberOfIterations = solver.solve(solution);
      assertEquals(1, numberOfIterations);

      assertEquals(1, solution.length);
      assertEquals(-1.0, solution[0], epsilon);

      // Minimize x^T * x subject to 1 + epsilon <= x <= 1 - epsilon (Should not give valid solution since this is too much to blame on roundoff)
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0 } };
      costLinearVector = new double[] { 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      variableLowerBounds = new double[] { 1.0 + 1e7 };
      variableUpperBounds = new double[] { 1.0 - 1e7 };
      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      solution = new double[1];
      boolean caughtException = false;
      try
      {
         numberOfIterations = solver.solve(solution);
      }
      catch (NoConvergenceException e)
      {
         caughtException = true;
      }
      assertEquals(1, numberOfIterations);

      assertTrue(caughtException);
      assertEquals(1, solution.length);
      assertTrue(Double.isNaN(solution[0]));

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, -5 <= x <= 5, 6 <= y <= 10, 11 <= z
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { -5.0, 6.0, 11.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      solution = new double[3];

      solver.solve(solution);
      numberOfIterations = solver.solve(solution);
      assertEquals(3, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(-4.0, solution[0], epsilon);
      assertEquals(6.0, solution[1], epsilon);
      assertEquals(14.0, solution[2], epsilon);

      DenseMatrix64F solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(248.0, objectiveCost, epsilon);

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, 3 <= x <= 5, 6 <= y <= 10, 11 <= z
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { 3.0, 6.0, 11.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      solution = new double[3];

      caughtException = false;
      try
      {
         solver.solve(solution);
      }
      catch (NoConvergenceException e)
      {
         caughtException = true;
      }
      assertTrue(caughtException);
      caughtException = false;
      try
      {
         numberOfIterations = solver.solve(solution);
      }
      catch (NoConvergenceException e)
      {
         caughtException = true;
         numberOfIterations = e.getIter();
      }
      assertTrue(caughtException);
      assertEquals(2, numberOfIterations); //// TODO: 5/15/17

      assertEquals(3, solution.length);
      assertTrue(Double.isNaN(solution[0]));
      assertTrue(Double.isNaN(solution[1]));
      assertTrue(Double.isNaN(solution[2]));

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertTrue(Double.isNaN(objectiveCost));
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testClear() throws NoConvergenceException
   {
      JavaQuadProgSolver solver = new JavaQuadProgSolver();

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, -5 <= x <= 5, 6 <= y <= 10, 11 <= z
      solver.clear();

      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      double[] costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { -5.0, 6.0, 11.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY }); //todo

      double[] solution = new double[3];

      solver.solve(solution);
      int numberOfIterations = solver.solve(solution);
      assertEquals(3, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(-4.0, solution[0], epsilon);
      assertEquals(6.0, solution[1], epsilon);
      assertEquals(14.0, solution[2], epsilon);

      DenseMatrix64F solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(248.0, objectiveCost, epsilon);

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8  (Remove -5 <= x <= 5, 6 <= y <= 10, 11 <= z)
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solution = new double[3];
      solver.solve(solution);
      numberOfIterations = solver.solve(solution);
      assertEquals(1, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(4.0, solution[0], epsilon);
      assertEquals(-2.0, solution[1], epsilon);
      assertEquals(6.0, solution[2], epsilon);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(56.0, objectiveCost, epsilon);

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, (Remove y - z <= -8)
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      solution = new double[3];

      solver.solve(solution);
      numberOfIterations = solver.solve(solution);
      assertEquals(0, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(1.0, solution[0], epsilon);
      assertEquals(1.0, solution[1], epsilon);
      assertEquals(0.0, solution[2], epsilon);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(2.0, objectiveCost, epsilon);

      // Minimize x^2 + y^2 + z^2 (Remove subject to x + y = 2.0)
      solver.clear();

      costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      solution = new double[3];

      solver.solve(solution);
      numberOfIterations = solver.solve(solution);
      assertEquals(0, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(0.0, solution[0], epsilon);
      assertEquals(0.0, solution[1], epsilon);
      assertEquals(0.0, solution[2], epsilon);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(0.0, objectiveCost, epsilon);

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0 (Added without clearing)
      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      solution = new double[3];

      solver.solve(solution);
      numberOfIterations = solver.solve(solution);
      assertEquals(0, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(1.0, solution[0], epsilon);
      assertEquals(1.0, solution[1], epsilon);
      assertEquals(0.0, solution[2], epsilon);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(2.0, objectiveCost, epsilon);

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8  (Added without clearing)
      linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solution = new double[3];

      solver.solve(solution);
      numberOfIterations = solver.solve(solution);
      assertEquals(1, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(4.0, solution[0], epsilon);
      assertEquals(-2.0, solution[1], epsilon);
      assertEquals(6.0, solution[2], epsilon);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(56.0, objectiveCost, epsilon);

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, -5 <= x <= 5, 6 <= y <= 10, 11 <= z (Added without clearing)
      solver.setVariableBounds(new double[] { -5.0, 6.0, 11.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      solution = new double[3];

      solver.solve(solution);
      numberOfIterations = solver.solve(solution);
      assertEquals(3, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(-4.0, solution[0], epsilon);
      assertEquals(6.0, solution[1], 1e-7);
      assertEquals(14.0, solution[2], 1e-7);

      solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(248.0, objectiveCost, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSolutionMethodsAreAllConsistent() throws NoConvergenceException
   {
      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      QuadProgSolver originalSolver = new QuadProgSolver();

      // Minimize x^2 + y^2 subject to x + y = 2.0, y >= 0.5, y >= 3.0, y >= x-3  (-y <= -0.5, -y <= -3.0, x - y <= 3
      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      double[] costLinearVector = new double[] { 0.0, 0.0 };
      double quadraticCostScalar = 0.0;
      DenseMatrix64F G = new DenseMatrix64F(costQuadraticMatrix);
      DenseMatrix64F f = MatrixTools.createVector(costLinearVector);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 2.0 };
      DenseMatrix64F Aeq = new DenseMatrix64F(linearEqualityConstraintsAMatrix);
      DenseMatrix64F beq = MatrixTools.createVector(linearEqualityConstraintsBVector);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 0.0, -1.0 }, { 0.0, -1.0 }, { 1.0, -1.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { -0.5, -3.0, 3.0 };
      DenseMatrix64F Ain = new DenseMatrix64F(linearInequalityConstraintsCMatrix);
      DenseMatrix64F bin = MatrixTools.createVector(linearInqualityConstraintsDVector);

      DenseMatrix64F solutionMatrix = new DenseMatrix64F(2, 1);
      DenseMatrix64F solutionMatrixAlt = new DenseMatrix64F(2, 1);

      solver.clear();
      solver.setQuadraticCostFunction(G, f, quadraticCostScalar);
      solver.setLinearEqualityConstraints(Aeq, beq);
      solver.setLinearInequalityConstraints(Ain, bin);
      int numberOfIterations = solver.solve(solutionMatrix);
      originalSolver.solve(G, f, Aeq, beq, Ain, bin, solutionMatrixAlt, false);

      assertEquals(1, numberOfIterations);

      assertEquals(-1.0, solutionMatrix.get(0), 1e-7);
      assertEquals(3.0, solutionMatrix.get(1), 1e-7);
      assertEquals(-1.0, solutionMatrixAlt.get(0), 1e-7);
      assertEquals(3.0, solutionMatrixAlt.get(1), 1e-7);

      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(10.0, objectiveCost, 1e-7);

      // Try with other solve method:
      double[] solution = new double[2];
      solver.clear();
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);
      numberOfIterations = solver.solve(solution);

      assertEquals(1, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(-1.0, solution[0], 1e-7);
      assertEquals(3.0, solution[1], 1e-7);

      solutionMatrix.setData(solution);
      objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(10.0, objectiveCost, 1e-7);

      // Try with other solve method:
      solver.clear();
      DenseMatrix64F quadraticCostMatrix64F = new DenseMatrix64F(costQuadraticMatrix);
      DenseMatrix64F linearCostVector64F = new DenseMatrix64F(costLinearVector.length, 1);
      linearCostVector64F.setData(costLinearVector);

      solver.setQuadraticCostFunction(quadraticCostMatrix64F, linearCostVector64F, quadraticCostScalar);
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      DenseMatrix64F solutionMatrix64F = new DenseMatrix64F(quadraticCostMatrix64F.getNumRows(), 1);
      numberOfIterations = solver.solve(solutionMatrix64F);

      assertEquals(1, numberOfIterations);

      assertEquals(2, solutionMatrix64F.getNumRows());
      assertEquals(-1.0, solutionMatrix64F.get(0, 0), 1e-7);
      assertEquals(3.0, solutionMatrix64F.get(1, 0), 1e-7);

      objectiveCost = solver.getObjectiveCost(solutionMatrix64F);
      assertEquals(10.0, objectiveCost, 1e-7);

      // Try with other solve method:
      solver = new JavaQuadProgSolver();

      solver.setQuadraticCostFunction(quadraticCostMatrix64F, linearCostVector64F, quadraticCostScalar);
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      DenseMatrix64F linearInequalityConstraintsCMatrix64F = new DenseMatrix64F(linearInequalityConstraintsCMatrix);
      DenseMatrix64F linearInqualityConstraintsDVector64F = new DenseMatrix64F(linearInqualityConstraintsDVector.length, 1);
      linearInqualityConstraintsDVector64F.setData(linearInqualityConstraintsDVector);
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix64F, linearInqualityConstraintsDVector64F);

      solutionMatrix64F.zero();
      numberOfIterations = solver.solve(solutionMatrix64F);

      assertEquals(1, numberOfIterations);

      assertEquals(2, solutionMatrix64F.getNumRows());
      assertEquals(-1.0, solutionMatrix64F.get(0, 0), 1e-7);
      assertEquals(3.0, solutionMatrix64F.get(1, 0), 1e-7);

      objectiveCost = solver.getObjectiveCost(solutionMatrix64F);
      assertEquals(10.0, objectiveCost, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test2DCasesWithPolygonConstraints() throws NoConvergenceException
   {
      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      QuadProgSolver originalSolver = new QuadProgSolver();

      // Minimize x^2 + y^2 subject to 3 <= x <= 5, 2 <= y <= 4
      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      double[] costLinearVector = new double[] { 0.0, 0.0 };
      double quadraticCostScalar = 0.0;
      DenseMatrix64F Q = new DenseMatrix64F(costQuadraticMatrix);
      DenseMatrix64F f = MatrixTools.createVector(costLinearVector);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 0.0 }, { -1.0, 0.0 }, { 0.0, 1.0 }, { 0.0, -1.0 } };
      double[] linearInequalityConstraintsDVector = new double[] { 5.0, -3.0, 4.0, -2.0 };
      DenseMatrix64F Ain = new DenseMatrix64F(linearInequalityConstraintsCMatrix);
      DenseMatrix64F bin = MatrixTools.createVector(linearInequalityConstraintsDVector);

      DenseMatrix64F Aeq = new DenseMatrix64F(0, 2);
      DenseMatrix64F beq = new DenseMatrix64F(0, 1);

      DenseMatrix64F solutionMatrix = new DenseMatrix64F(2, 1);
      DenseMatrix64F solutionMatrixAlt = new DenseMatrix64F(2, 1);
      solver.clear();
      solver.setQuadraticCostFunction(Q, f, quadraticCostScalar);
      solver.setLinearInequalityConstraints(Ain, bin);
      solver.setLinearEqualityConstraints(Aeq, beq);
      int numberOfIterations = solver.solve(solutionMatrix);
      originalSolver.solve(Q, f, Aeq, beq, Ain, bin, solutionMatrixAlt, false);

      assertEquals(2, numberOfIterations);

      assertEquals(3.0, solutionMatrixAlt.get(0), 1e-7);
      assertEquals(2.0, solutionMatrixAlt.get(1), 1e-7);
      assertEquals(3.0, solutionMatrix.get(0), 1e-7);
      assertEquals(2.0, solutionMatrix.get(1), 1e-7);

      // Minimize x^2 + y^2 subject to x + y >= 2 (-x -y <= -2), y <= 10x - 2 (-10x + y <= -2)
      // Equality solution will violate both constraints, but optimal only has the first constraint active.
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      Q = new DenseMatrix64F(costQuadraticMatrix);
      f = MatrixTools.createVector(costLinearVector);

      linearInequalityConstraintsCMatrix = new double[][] { { -1.0, -1.0 }, { -10.0, 1.0 } };
      linearInequalityConstraintsDVector = new double[] { -2.0, -2.0 };
      Ain = new DenseMatrix64F(linearInequalityConstraintsCMatrix);
      bin = MatrixTools.createVector(linearInequalityConstraintsDVector);

      Aeq = new DenseMatrix64F(0, 2);
      beq = new DenseMatrix64F(0, 1);

      solver.clear();
      solver.setQuadraticCostFunction(Q, f, quadraticCostScalar);
      solver.setLinearInequalityConstraints(Ain, bin);
      solver.setLinearEqualityConstraints(Aeq, beq);
      numberOfIterations = solver.solve(solutionMatrix);
      originalSolver.solve(Q, f, Aeq, beq, Ain, bin, solutionMatrixAlt, false);
      assertEquals(1, numberOfIterations);

      assertEquals(1.0, solutionMatrixAlt.get(0), 1e-7);
      assertEquals(1.0, solutionMatrixAlt.get(1), 1e-7);
      assertEquals(1.0, solutionMatrix.get(0), 1e-7);
      assertEquals(1.0, solutionMatrix.get(1), 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testChallengingCasesWithPolygonConstraints() throws NoConvergenceException
   {
      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      solver.setMaxNumberOfIterations(10);

      // Minimize x^2 + y^2 subject to x + y >= 2 (-x -y <= -2), y <= 10x - 2 (-10x + y <= -2), x <= 10y - 2 (x - 10y <= -2),
      // Equality solution will violate all three constraints, but optimal only has the first constraint active.
      // However, if you set all three constraints active, there is no solution.
      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      double[] costLinearVector = new double[] { 0.0, 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { -1.0, -1.0 }, { -10.0, 1.0 }, { 1.0, -10.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { -2.0, -2.0, -2.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      double[] solution = new double[2];
      int numberOfIterations = solver.solve(solution);
      assertEquals(1, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(1.0, solution[0], 1e-7);
      assertEquals(1.0, solution[1], 1e-7);

      // Reorder and make sure it works:
      // Minimize x^2 + y^2 subject to x + y >= 2 (-x -y <= -2), y <= 10x - 2 (-10x + y <= -2), x <= 10y - 2 (x - 10y <= -2),
      // Equality solution will violate all three constraints, but optimal only has the first constraint active.
      // However, if you set all three constraints active, there is no solution.
      solver.clear();
      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      costLinearVector = new double[] { 0.0, 0.0 };
      quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      linearInequalityConstraintsCMatrix = new double[][] { { -10.0, 1.0 }, { -1.0, -1.0 }, { 1.0, -10.0 } };
      linearInqualityConstraintsDVector = new double[] { -2.0, -2.0, -2.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solution = new double[2];
      numberOfIterations = solver.solve(solution);
      assertEquals(3, numberOfIterations);

      assertEquals(2, solution.length);
      assertEquals(1.0, solution[0], 0.06);
      assertEquals(1.0, solution[1], 0.06);
   }

   // This should pass with a good solver. But a simple one has trouble on it.
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testChallengingCasesWithPolygonConstraintsCheckFailsWithSimpleSolver() throws NoConvergenceException
   {
      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      solver.setMaxNumberOfIterations(10);

      // Minimize x^2 + y^2 subject to x + y >= 2 (-x -y <= -2), y <= 10x - 2 (-10x + y <= -2), x <= 10y - 2 (x - 10y <= -2),
      // Equality solution will violate all three constraints, but optimal only has the first constraint active.
      // However, if you set all three constraints active, there is no solution.
      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      double[] costLinearVector = new double[] { 0.0, 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { -1.0, -1.0 }, { -10.0, 1.0 }, { 1.0, -10.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { -2.0, -2.0, -2.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      double[] solution = new double[2];
      solver.solve(solution);

      assertEquals(2, solution.length);
      assertEquals(1.0, solution[0], 1e-7);
      assertEquals(1.0, solution[1], 1e-7);
   }

   // this has no solution with a simple solver
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCaseWithNoSolution() throws NoConvergenceException
   {
      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      int maxNumberOfIterations = 10;
      solver.setMaxNumberOfIterations(maxNumberOfIterations);

      // Minimize x^2 + y^2 subject to x + y = 5, x + y <= 2
      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      double[] costLinearVector = new double[] { 0.0, 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 5.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 1.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { 2.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      double[] solution = new double[2];
      boolean caughtException = false;
      try
      {
         solver.solve(solution);
      }
      catch (NoConvergenceException e)
      {
         caughtException = true;
      }
      assertTrue(caughtException);

      assertEquals(2, solution.length);
      assertTrue(Double.isNaN(solution[0]));
      assertTrue(Double.isNaN(solution[1]));
   }

   @Ignore
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testLargeRandomProblemWithInequalityConstraints() throws NoConvergenceException
   {
      Random random = new Random(1776L);

      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      QuadProgSolver originalSolver = new QuadProgSolver();

      int numberOfTests = 100;

      long startTimeMillis = System.currentTimeMillis();
      int maxNumberOfIterations = 0;

      int numberOfVariables = 80;
      int numberOfEqualityConstraints = 10;
      int numberOfInequalityConstraints = 36;

      DenseMatrix64F solution = new DenseMatrix64F(numberOfVariables, 1);
      DenseMatrix64F solutionAlt = new DenseMatrix64F(numberOfVariables, 1);
      double[] solutionWithSmallPerturbation = new double[numberOfVariables];

      DenseMatrix64F augmentedLinearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
      DenseMatrix64F augmentedLinearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);

      for (int testNumber = 0; testNumber < numberOfTests; testNumber++)
      {
         solver.clear();

         DenseMatrix64F costQuadraticMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, numberOfVariables);
         DenseMatrix64F identity = CommonOps.identity(numberOfVariables, numberOfVariables); // Add n*I to make sure it is positive definite...
         CommonOps.scale(numberOfVariables, identity);
         CommonOps.addEquals(costQuadraticMatrix, identity);

         DenseMatrix64F costLinearVector = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, 1);
         double quadraticCostScalar = RandomNumbers.nextDouble(random, 30.0);

         solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

         DenseMatrix64F linearEqualityConstraintsAMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfEqualityConstraints, numberOfVariables);
         DenseMatrix64F linearEqualityConstraintsBVector = RandomGeometry.nextDenseMatrix64F(random, numberOfEqualityConstraints, 1);
         solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

         DenseMatrix64F linearInequalityConstraintsCMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfInequalityConstraints, numberOfVariables);
         DenseMatrix64F linearInequalityConstraintsDVector = RandomGeometry.nextDenseMatrix64F(random, numberOfInequalityConstraints, 1);
         solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector);

         int numberOfIterations = solver.solve(solution);
         originalSolver.solve(costQuadraticMatrix, costLinearVector, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solutionAlt, false);
         if (numberOfIterations > maxNumberOfIterations)
            maxNumberOfIterations = numberOfIterations;
         //         System.out.println("numberOfIterations = " + numberOfIterations);

         assertEquals(numberOfVariables, solution.getNumRows());

         double objectiveCost = solver.getObjectiveCost(solution);

         // Verify equality constraints hold:
         verifyEqualityConstraintsHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solution);

         // Verify objective is minimized by comparing to small perturbation:
         for (int i = 0; i < numberOfVariables; i++)
         {
            solutionWithSmallPerturbation[i] = solution.get(i, 0) + RandomNumbers.nextDouble(random, 1e-4);
         }

         solution.zero();
         solution.setData(solutionWithSmallPerturbation);

         verifyEqualityConstraintsDoNotHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solution);

         // Equality constraints usually do not hold. Sometimes they do, so if you run with lots of numberOfTests, comment out the following:
         verifyInequalityConstraintsDoNotHold(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solution);

         int activeSetSize = 0;

         augmentedLinearEqualityConstraintsAMatrix.reshape(numberOfEqualityConstraints + activeSetSize, numberOfVariables);
         augmentedLinearEqualityConstraintsBVector.reshape(numberOfEqualityConstraints + activeSetSize, 1);

         CommonOps.extract(linearEqualityConstraintsAMatrix, 0, numberOfEqualityConstraints, 0, numberOfVariables, augmentedLinearEqualityConstraintsAMatrix, 0, 0);
         CommonOps.extract(linearEqualityConstraintsBVector, 0, numberOfEqualityConstraints, 0, 1, augmentedLinearEqualityConstraintsBVector, 0, 0);

         DenseMatrix64F solutionMatrixProjectedOntoEqualityConstraints = projectOntoEqualityConstraints(solution, augmentedLinearEqualityConstraintsAMatrix, augmentedLinearEqualityConstraintsBVector);
         verifyEqualityConstraintsHold(numberOfEqualityConstraints + activeSetSize, augmentedLinearEqualityConstraintsAMatrix, augmentedLinearEqualityConstraintsBVector, solutionMatrixProjectedOntoEqualityConstraints);

         double objectiveCostWithSmallPerturbation = solver.getObjectiveCost(solutionMatrixProjectedOntoEqualityConstraints);

         assertTrue("objectiveCostWithSmallPerturbation = " + objectiveCostWithSmallPerturbation + ", objectiveCost = " + objectiveCost, objectiveCostWithSmallPerturbation > objectiveCost);
      }

      long endTimeMillis = System.currentTimeMillis();

      double timePerTest = ((double) (endTimeMillis - startTimeMillis)) * 0.001 / ((double) numberOfTests);
      if (VERBOSE)
      {
         System.out.println("Time per test is " + timePerTest);
         System.out.println("maxNumberOfIterations is " + maxNumberOfIterations);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testLargeRandomProblemWithInequalityAndBoundsConstraints() throws NoConvergenceException
   {
      Random random = new Random(1776L);

      JavaQuadProgSolver solver = new JavaQuadProgSolver();

      int numberOfTests = 100;

      long startTimeMillis = System.currentTimeMillis();
      int maxNumberOfIterations = 0;

      int numberOfVariables = 80;
      int numberOfEqualityConstraints = 10;
      int numberOfInequalityConstraints = 36;

      DenseMatrix64F solution = new DenseMatrix64F(0, 0);
      double[] solutionWithSmallPerturbation = new double[numberOfVariables];

      int numberOfNaNSolutions = 0;
      for (int testNumber = 0; testNumber < numberOfTests; testNumber++)
      {
         solver.clear();

         DenseMatrix64F costQuadraticMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, numberOfVariables);
         DenseMatrix64F identity = CommonOps.identity(numberOfVariables, numberOfVariables); // Add n*I to make sure it is positive definite...
         CommonOps.scale(numberOfVariables, identity);
         CommonOps.addEquals(costQuadraticMatrix, identity);

         DenseMatrix64F costLinearVector = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, 1);
         double quadraticCostScalar = RandomNumbers.nextDouble(random, 30.0);

         solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

         DenseMatrix64F linearEqualityConstraintsAMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfEqualityConstraints, numberOfVariables);
         DenseMatrix64F linearEqualityConstraintsBVector = RandomGeometry.nextDenseMatrix64F(random, numberOfEqualityConstraints, 1);
         solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

         DenseMatrix64F linearInequalityConstraintsCMatrix = RandomGeometry.nextDenseMatrix64F(random, numberOfInequalityConstraints, numberOfVariables);
         DenseMatrix64F linearInequalityConstraintsDVector = RandomGeometry.nextDenseMatrix64F(random, numberOfInequalityConstraints, 1);
         solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector);

         DenseMatrix64F variableLowerBounds = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, 1, -5.0, -0.01);
         DenseMatrix64F variableUpperBounds = RandomGeometry.nextDenseMatrix64F(random, numberOfVariables, 1, 0.01, 5.0);
         solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

         int numberOfIterations = solver.solve(solution);
         if (numberOfIterations > maxNumberOfIterations)
            maxNumberOfIterations = numberOfIterations;


         assertEquals(numberOfVariables, solution.getNumRows());

         if (Double.isNaN(solution.get(0)))
         {
            numberOfNaNSolutions++;
            continue;
         }

         // Verify equality constraints hold:
         verifyEqualityConstraintsHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solution);

         // Verify objective is minimized by comparing to small perturbation:
         for (int i = 0; i < numberOfVariables; i++)
         {
            solutionWithSmallPerturbation[i] = solution.get(i, 0) + RandomNumbers.nextDouble(random, 1e-4);
         }

         solution.zero();
         solution.setData(solutionWithSmallPerturbation);

         verifyEqualityConstraintsDoNotHold(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solution);

         // Equality constraints usually do not hold. Sometimes they do, so if you run with lots of numberOfTests, comment out the following:
         verifyInequalityConstraintsDoNotHold(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solution);
      }

      assertTrue(numberOfNaNSolutions < 0.05 * numberOfTests);

      long endTimeMillis = System.currentTimeMillis();

      double timePerTest = ((double) (endTimeMillis - startTimeMillis)) * 0.001 / ((double) numberOfTests);
      if (VERBOSE)
      {
         System.out.println("Time per test is " + timePerTest);
         System.out.println("maxNumberOfIterations is " + maxNumberOfIterations);
         System.out.println("numberOfNaNSolutions = " + numberOfNaNSolutions);
         System.out.println("numberOfTests = " + numberOfTests);

      }
   }

   /**
    *  Test with dataset from sim that revealed a bug with the variable lower/upper bounds handling.
    */
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFindValidSolutionForDataset20160319() throws NoConvergenceException
   {
      ActualDatasetFrom20160319 dataset = new ActualDatasetFrom20160319();
      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      solver.clear();
      solver.setQuadraticCostFunction(dataset.getCostQuadraticMatrix(), dataset.getCostLinearVector(), 0.0);
      solver.setVariableBounds(dataset.getVariableLowerBounds(), dataset.getVariableUpperBounds());
      DenseMatrix64F solution = new DenseMatrix64F(dataset.getProblemSize(), 1);
      solver.solve(solution);

      assertTrue(!MatrixTools.containsNaN(solution));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMaxIterations() throws NoConvergenceException
   {
      JavaQuadProgSolver solver = new JavaQuadProgSolver();

      // Minimize x^2 + y^2 + z^2 subject to x + y = 2.0, y - z <= -8, -5 <= x <= 5, 6 <= y <= 10, 11 <= z
      solver.clear();

      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0, 0.0 }, { 0.0, 2.0, 0.0 }, { 0.0, 0.0, 2.0 } };
      double[] costLinearVector = new double[] { 0.0, 0.0, 0.0 };
      double quadraticCostScalar = 0.0;
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 1.0, 0.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 2.0 };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 0.0, 1.0, -1.0 } };
      double[] linearInqualityConstraintsDVector = new double[] { -8.0 };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

      solver.setVariableBounds(new double[] { -5.0, 6.0, 11.0 }, new double[] { 5.0, 10.0, Double.POSITIVE_INFINITY });

      double[] solution = new double[3];

      solver.setMaxNumberOfIterations(2);
      int numberOfIterations;
      try
      {
         numberOfIterations = solver.solve(solution);
      }
      catch (NoConvergenceException e)
      {
         numberOfIterations = e.getIter();
      }
      assertEquals(2, numberOfIterations);

      assertTrue(Double.isNaN(solution[0]));
      assertTrue(Double.isNaN(solution[1]));
      assertTrue(Double.isNaN(solution[2]));

      solver.setMaxNumberOfIterations(4);
      try
      {
         numberOfIterations = solver.solve(solution);
      }
      catch (NoConvergenceException e)
      {
         numberOfIterations = e.getIter();
      }
      assertEquals(3, numberOfIterations);

      assertEquals(3, solution.length);
      assertEquals(-4.0, solution[0], 1e-7);
      assertEquals(6.0, solution[1], 1e-7);
      assertEquals(14.0, solution[2], 1e-7);

      DenseMatrix64F solutionMatrix = new DenseMatrix64F(costQuadraticMatrix.length, 1);
      solutionMatrix.setData(solution);
      double objectiveCost = solver.getObjectiveCost(solutionMatrix);
      assertEquals(248.0, objectiveCost, 1e-7);
   }

   /*
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeExceptions()
   {
      SimpleActiveSetQPSolverInterface solver = createSolverToTest();

      double[][] costQuadraticMatrix = new double[][] { { 2.0, 0.0 } };
      double[] costLinearVector = new double[] { 0.0 };
      double quadraticCostScalar = 0.0;

      try
      {
         solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);
         fail("costQuadraticMatrix needs to be square!");
      }
      catch(RuntimeException e)
      {
      }

      costQuadraticMatrix = new double[][] { { 2.0, 0.0 }, { 0.0, 2.0 } };
      try
      {
         solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);
         fail("costQuadraticMatrix needs to be same length as costLinearVector!");
      }
      catch(RuntimeException e)
      {
      }

      try
      {
         DenseMatrix64F costQuadraticMatrix64F = new DenseMatrix64F(2, 2);
         DenseMatrix64F costLinearVector64F = new DenseMatrix64F(1, 2);

         solver.setQuadraticCostFunction(costQuadraticMatrix64F, costLinearVector64F, quadraticCostScalar);
         fail("Wrong size for costLinearVector64F.");
      }
      catch(RuntimeException e)
      {
      }

      costLinearVector = new double[] { 0.0, 0.0 };
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);

      // Variable Bounds
      double[] variableLowerBounds = new double[]{0.0};
      double[] variableUpperBounds = new double[]{0.0, 0.0};

      try
      {
         solver.setVariableBounds(variableLowerBounds, variableUpperBounds);
         fail("Wrong lower bounds size");
      }
      catch(RuntimeException e)
      {
      }

      variableLowerBounds = new double[]{0.0, 0.0};
      variableUpperBounds = new double[]{0.0};

      try
      {
         solver.setVariableBounds(variableLowerBounds, variableUpperBounds);
         fail("Wrong upper bounds size");
      }
      catch(RuntimeException e)
      {
      }

      variableLowerBounds = new double[]{0.0, 0.0};
      variableUpperBounds = new double[]{0.0, 0.0};

      solver.setVariableBounds(variableLowerBounds, variableUpperBounds);

      // Equality Constraints
      double[][] linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 0.0 } };
      double[] linearEqualityConstraintsBVector = new double[] { 1.0, 2.0 };

      try
      {
         solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);
         fail("Wrong size for linearEqualityConstraintsBVector.");
      }
      catch(RuntimeException e)
      {
      }

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0 } };
      linearEqualityConstraintsBVector = new double[] { 1.0 };
      try
      {
         solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);
         fail("Wrong size for linearEqualityConstraintsBVector.");
      }
      catch(RuntimeException e)
      {
      }

      try
      {
         DenseMatrix64F linearEqualityConstraintsAMatrix64F = new DenseMatrix64F(2, 2);
         DenseMatrix64F linearEqualityConstraintsBVector64F = new DenseMatrix64F(1, 2);

         solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix64F, linearEqualityConstraintsBVector64F);
         fail("Wrong size for linearEqualityConstraintsBVector.");
      }
      catch(RuntimeException e)
      {
      }

      linearEqualityConstraintsAMatrix = new double[][] { { 1.0, 0.0 } };
      solver.setLinearEqualityConstraints(linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector);

      // Inequality Constraints
      double[][] linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 0.0 } };
      double[] linearInequalityConstraintsDVector = new double[] { 1.0, 2.0 };

      try
      {
         solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector);
         fail("Wrong size for linearInqualityConstraintsDVector.");
      }
      catch(RuntimeException e)
      {
      }

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0 } };
      linearInequalityConstraintsDVector = new double[] { 1.0 };
      try
      {
         solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector);
         fail("Wrong size for linearInqualityConstraintsDVector.");
      }
      catch(RuntimeException e)
      {
      }

      try
      {
         DenseMatrix64F linearInequalityConstraintsCMatrix64F = new DenseMatrix64F(2, 2);
         DenseMatrix64F linearInequalityConstraintsDVector64F = new DenseMatrix64F(1, 2);

         solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix64F, linearInequalityConstraintsDVector64F);
         fail("Wrong size for linearInequalityConstraintsDVector64F.");
      }
      catch(RuntimeException e)
      {
      }

      linearInequalityConstraintsCMatrix = new double[][] { { 1.0, 0.0 } };
      solver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector);


      double[] solution = new double[10];
      double[] lagrangeEqualityMultipliers = new double[1];
      double[] lagrangeInequalityMultipliers = new double[1];

      try
      {
         solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
         fail("Wrong size to pack");
      }
      catch(RuntimeException e)
      {
      }

      solution = new double[2];
      lagrangeEqualityMultipliers = new double[10];
      try
      {
         solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
         fail("Wrong size of lagrangeEqualityMultipliersto pack");
      }
      catch(RuntimeException e)
      {
      }

      lagrangeEqualityMultipliers = new double[1];
      lagrangeInequalityMultipliers = new double[10];
      try
      {
         solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);
         fail("Wrong size of lagrangeInequalityMultipliers pack");
      }
      catch(RuntimeException e)
      {
      }

      lagrangeInequalityMultipliers = new double[1];

      double[] lowerBoundMultipliers = new double[10];
      double[] upperBoundMultipliers = new double[2];
      try
      {
         solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lowerBoundMultipliers, upperBoundMultipliers);
         fail("Wrong size of lowerBoundMultipliers to pack");
      }
      catch(RuntimeException e)
      {
      }

      lowerBoundMultipliers = new double[2];
      upperBoundMultipliers = new double[10];
      try
      {
         solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lowerBoundMultipliers, upperBoundMultipliers);
         fail("Wrong size of upperBoundMultipliers to pack");
      }
      catch(RuntimeException e)
      {
      }

      lowerBoundMultipliers = new double[2];
      upperBoundMultipliers = new double[2];

      solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers, lowerBoundMultipliers, upperBoundMultipliers);
   }
   */

   private void verifyEqualityConstraintsHold(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      double maxAbsoluteError = getMaxEqualityConstraintError(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);
      assertEquals(0.0, maxAbsoluteError, 1e-5);
   }

   private void verifyInequalityConstraintsHold(int numberOfEqualityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix, DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      double maxSignedError = getMaxInequalityConstraintError(numberOfEqualityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solutionMatrix);
      assertTrue(maxSignedError < 1e-10);
   }

   private void verifyEqualityConstraintsDoNotHold(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      double maxAbsoluteError = getMaxEqualityConstraintError(numberOfEqualityConstraints, linearEqualityConstraintsAMatrix, linearEqualityConstraintsBVector, solutionMatrix);
      assertTrue(maxAbsoluteError > 1e-5);
   }

   private void verifyInequalityConstraintsDoNotHold(int numberOfInequalityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix, DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      double maxError = getMaxInequalityConstraintError(numberOfInequalityConstraints, linearInequalityConstraintsCMatrix, linearInequalityConstraintsDVector, solutionMatrix);
      assertTrue(maxError > 1e-5);
   }

   private void verifyVariableBoundsHold(DenseMatrix64F variableLowerBounds, DenseMatrix64F variableUpperBounds, DenseMatrix64F solution)
   {
      for (int i = 0; i < variableLowerBounds.getNumRows(); i++)
      {
         assertTrue(solution.get(i, 0) >= variableLowerBounds.get(i, 0) - 1e-7);
      }

      for (int i = 0; i < variableUpperBounds.getNumRows(); i++)
      {
         assertTrue(solution.get(i, 0) <= variableUpperBounds.get(i, 0) + 1e-7);
      }
   }

   private double getMaxEqualityConstraintError(int numberOfEqualityConstraints, DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector, DenseMatrix64F solutionMatrix)
   {
      DenseMatrix64F checkMatrix = new DenseMatrix64F(numberOfEqualityConstraints, 1);
      CommonOps.mult(linearEqualityConstraintsAMatrix, solutionMatrix, checkMatrix);
      CommonOps.subtractEquals(checkMatrix, linearEqualityConstraintsBVector);

      return getMaxAbsoluteDataEntry(checkMatrix);
   }

   private double getMaxInequalityConstraintError(int numberOfInequalityConstraints, DenseMatrix64F linearInequalityConstraintsCMatrix, DenseMatrix64F linearInequalityConstraintsDVector, DenseMatrix64F solutionMatrix)
   {
      DenseMatrix64F checkMatrix = new DenseMatrix64F(numberOfInequalityConstraints, 1);
      CommonOps.mult(linearInequalityConstraintsCMatrix, solutionMatrix, checkMatrix);
      CommonOps.subtractEquals(checkMatrix, linearInequalityConstraintsDVector);

      return getMaxSignedDataEntry(checkMatrix);
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

   private double getMaxSignedDataEntry(DenseMatrix64F matrix)
   {
      int numberOfRows = matrix.getNumRows();
      int numberOfColumns = matrix.getNumCols();

      double max = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < numberOfRows; i++)
      {
         for (int j = 0; j < numberOfColumns; j++)
         {
            double value = matrix.get(i, j);
            if (value > max)
            {
               max = value;
            }
         }
      }

      return max;
   }
}

