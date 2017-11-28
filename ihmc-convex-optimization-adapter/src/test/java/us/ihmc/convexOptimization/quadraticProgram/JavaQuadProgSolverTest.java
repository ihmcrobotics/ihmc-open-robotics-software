package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;

import static org.junit.Assert.assertEquals;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class JavaQuadProgSolverTest extends AbstractSimpleActiveSetQPSolverTest
{
   private static final double epsilon = 1e-4;

   @Override
   public SimpleActiveSetQPSolverInterface createSolverToTest()
   {
      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      solver.setUseWarmStart(false);
      return solver;
   }

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
   public void testTimingAgainstSimpleSolver()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      ExecutionTimer quadProgTotalTimer = new ExecutionTimer("quadProgTotalTimer", registry);
      ExecutionTimer simpleTotalTimer = new ExecutionTimer("simpleTotalTimer", registry);
      ExecutionTimer quadProgTimer = new ExecutionTimer("quadProgTimer", registry);
      ExecutionTimer simpleTimer = new ExecutionTimer("simpleTimer", registry);

      // Minimize x^2 + y^2 subject to x + y >= 2 (-x -y <= -2), y <= 10x - 2 (-10x + y <= -2), x <= 10y - 2 (x - 10y <= -2),
      // Equality solution will violate all three constraints, but optimal only has the first constraint active.
      // However, if you set all three constraints active, there is no solution.
      double[][] costQuadraticMatrix = new double[][] {{2.0, 0.0}, {0.0, 2.0}};
      double[] costLinearVector = new double[] {0.0, 0.0};
      double quadraticCostScalar = 0.0;

      double[][] linearInequalityConstraintsCMatrix = new double[][] {{-1.0, -1.0}, {-10.0, 1.0}, {1.0, -10.0}};
      double[] linearInqualityConstraintsDVector = new double[] {-2.0, -2.0, -2.0};

      JavaQuadProgSolver quadProg = new JavaQuadProgSolver();
      SimpleEfficientActiveSetQPSolver simpleSolver = new SimpleEfficientActiveSetQPSolver();

      double[] quadProgSolution = new double[2];
      double[] quadProgLagrangeEqualityMultipliers = new double[0];
      double[] quadProgLagrangeInequalityMultipliers = new double[3];
      double[] simpleSolution = new double[2];
      double[] simpleLagrangeEqualityMultipliers = new double[0];
      double[] simpleLagrangeInequalityMultipliers = new double[3];

      for (int repeat = 0; repeat < 5000; repeat++)
      {
         quadProgTotalTimer.startMeasurement();

         quadProg.clear();
         quadProg.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);
         quadProg.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

         quadProgTimer.startMeasurement();
         quadProg.solve(quadProgSolution, quadProgLagrangeEqualityMultipliers, quadProgLagrangeInequalityMultipliers);
         quadProgTimer.stopMeasurement();

         quadProgTotalTimer.stopMeasurement();


         simpleTotalTimer.startMeasurement();

         simpleSolver.clear();
         simpleSolver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar);
         simpleSolver.setLinearInequalityConstraints(linearInequalityConstraintsCMatrix, linearInqualityConstraintsDVector);

         simpleTimer.startMeasurement();
         simpleSolver.solve(simpleSolution, simpleLagrangeEqualityMultipliers, simpleLagrangeInequalityMultipliers);
         simpleTimer.stopMeasurement();

         simpleTotalTimer.stopMeasurement();
      }

      PrintTools.info("Quad Prog total time : " + quadProgTotalTimer.getAverageTime());
      PrintTools.info("Simple total time : " + simpleTotalTimer.getAverageTime());
      PrintTools.info("Quad Prog solve time : " + quadProgTimer.getAverageTime());
      PrintTools.info("Simple solve time : " + simpleTimer.getAverageTime());
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

   @Override /** have to override because quad prog uses fewer iterations */
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSolutionMethodsAreAllConsistent() throws NoConvergenceException
   {
      testSolutionMethodsAreAllConsistent(1);
   }

   @Override /** have to override because quad prog uses fewer iterations */
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleCasesWithInequalityConstraints()
   {
      testSimpleCasesWithInequalityConstraints(0);
   }

   @Override /** have to override because quad prog uses fewer iterations */
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleCasesWithBoundsConstraints()
   {
      testSimpleCasesWithBoundsConstraints(0, 1, 2, 6, true);
   }

   @Override /** have to override because quad prog uses different iterations */
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testClear()
   {
      testClear(6, 1, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMaxIterations()
   {
      testMaxIterations(6, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test2DCasesWithPolygonConstraints()
   {
      test2DCasesWithPolygonConstraints(2, 1);
   }


   @Ignore
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testChallengingCasesWithPolygonConstraints()
   {
      testChallengingCasesWithPolygonConstraints(1, 5);
   }


   @Override /** This IS a good solver **/
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testChallengingCasesWithPolygonConstraintsCheckFailsWithSimpleSolver()
   {
      SimpleActiveSetQPSolverInterface solver = createSolverToTest();
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
      double[] lagrangeEqualityMultipliers = new double[0];
      double[] lagrangeInequalityMultipliers = new double[3];
      solver.solve(solution, lagrangeEqualityMultipliers, lagrangeInequalityMultipliers);

      assertEquals(2, solution.length);
      assertEquals(solution[0], 1.0, epsilon);
      assertEquals(solution[1], 1.0, epsilon);
      assertEquals(lagrangeInequalityMultipliers[0], 2.0, epsilon);
      assertEquals(lagrangeInequalityMultipliers[1], 0.0, epsilon);
      assertEquals(lagrangeInequalityMultipliers[2], 0.0, epsilon);
   }
}

