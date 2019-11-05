package us.ihmc.convexOptimization.quadraticProgram;

import static us.ihmc.robotics.Assert.assertEquals;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.testing.JUnitTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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

   @Test
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

   @Test
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

   @Test
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

   @Test
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
   @Test
   public void testSolutionMethodsAreAllConsistent() throws NoConvergenceException
   {
      testSolutionMethodsAreAllConsistent(1);
   }

   @Override /** have to override because quad prog uses fewer iterations */
   @Test
   public void testSimpleCasesWithInequalityConstraints()
   {
      testSimpleCasesWithInequalityConstraints(0);
   }

   @Override /** have to override because quad prog uses fewer iterations */
   @Test
   public void testSimpleCasesWithBoundsConstraints()
   {
      testSimpleCasesWithBoundsConstraints(0, 1, 6, 2, true);
   }

   @Override /** have to override because quad prog uses different iterations */
   @Test
   public void testClear()
   {
      testClear(6, 1, true);
   }

   @Override
   @Test
   public void testMaxIterations()
   {
      testMaxIterations(6, false);
   }

   @Override
   @Test
   public void test2DCasesWithPolygonConstraints()
   {
      test2DCasesWithPolygonConstraints(2, 1);
   }


   @Override
   @Disabled
   @Test
   public void testChallengingCasesWithPolygonConstraints()
   {
      testChallengingCasesWithPolygonConstraints(1, 5);
   }


   @Override /** This IS a good solver **/
   @Test
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

   /**
    * GW: exported this from a failing Atlas unit test 05/23/2019
    * Not sure if this should be solvable but in any case it should fail gracefully.
    */
   @Test
   public void testCaseFromSimulation()
   {
      DenseMatrix64F costQuadraticMatrix = new DenseMatrix64F(6, 6);
      costQuadraticMatrix.data = new double[] {993.9053988041245, 327.83942494534944, 993.556655887893, 327.83942494534944, 2308.09243287179, 354.1845700419416,
            327.83942494534944, 1423.124867640583, 327.83942494534944, 1422.7761247243516, 354.1845700419416, 2771.803937517132, 993.556655887893,
            327.83942494534944, 1009.1964870941272, 327.83942494534944, 2308.09243287179, 354.1845700419416, 327.83942494534944, 1422.7761247243516,
            327.83942494534944, 1438.4159559305858, 354.1845700419416, 2771.803937517132, 2308.09243287179, 354.1845700419416, 2308.09243287179,
            354.1845700419416, 5508.124706211761, 0.06581329118532331, 354.1845700419416, 2771.803937517132, 354.1845700419416, 2771.803937517132,
            0.06581329118532508, 5507.8812912972435};

      DenseMatrix64F costLinearVector = new DenseMatrix64F(6, 1);
      costLinearVector.data = new double[] {20222.5613016018, 5592.963753999038, 20222.5613016018, 5592.963753999038, 47486.04338938162, 5042.8181779521055};

      DenseMatrix64F quadraticCostScalar = new DenseMatrix64F(1, 1);
      quadraticCostScalar.data = new double[] {206999.16716064143};

      DenseMatrix64F linearInequalityConstraintCMatrix = new DenseMatrix64F(17, 6);
      linearInequalityConstraintCMatrix.data = new double[] {-0.532490759103742, 0.8464358165089191, 0.0, 0.0, 0.0, 0.0, 0.8781297702936439, 0.4784225188304082,
            0.0, 0.0, 0.0, 0.0, 0.4314826988044296, -0.9021212117184951, 0.0, 0.0, 0.0, 0.0, -0.8674617517025186, -0.4975038787117122, 0.0, 0.0, 0.0, 0.0,
            -0.532490759103742, 0.8464358165089191, -0.532490759103742, 0.8464358165089191, 0.0, 0.0, 0.8781297702936439, 0.4784225188304082,
            0.8781297702936439, 0.4784225188304082, 0.0, 0.0, 0.4314826988044296, -0.9021212117184951, 0.4314826988044296, -0.9021212117184951, 0.0, 0.0,
            -0.8674617517025186, -0.4975038787117122, -0.8674617517025186, -0.4975038787117122, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.9838237285970943,
            0.17913925044308668, 0.0, 0.0, 0.0, 0.0, -0.6119239483029338, 0.7909166084318549, 0.0, 0.0, 0.0, 0.0, -0.15126410326159784, 0.9884933844313095, 0.0,
            0.0, 0.0, 0.0, 0.5483141711347475, 0.836272425548526, 0.0, 0.0, 0.0, 0.0, 0.39417400487132703, -0.9190358284004487, 0.0, 0.0, 0.0, 0.0, -0.0, 1.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0};

      DenseMatrix64F linearInequalityConstraintDVector = new DenseMatrix64F(17, 1);
      linearInequalityConstraintDVector.data = new double[] {0.03209312106775908, 0.12675512800740218, 0.05477548585328318, 0.07957996960652647,
            0.07200972933235494, 0.15848182583418868, 0.09792941203582961, 0.1298875084279425, 8.725615888588424, 4.899796215838858, 0.8792506851639368,
            -4.6465724005639855, -2.4847821341512306, -0.8057687026687546, -9.103397051640995, 1.0993716410559957, 9.292818302213409};

      JavaQuadProgSolver solver = new JavaQuadProgSolver();
      solver.setMaxNumberOfIterations(100);
      solver.setUseWarmStart(true);
      solver.clear();
      solver.resetActiveConstraints();
      solver.setQuadraticCostFunction(costQuadraticMatrix, costLinearVector, quadraticCostScalar.get(0, 0));
      solver.setLinearInequalityConstraints(linearInequalityConstraintCMatrix, linearInequalityConstraintDVector);

      DenseMatrix64F solution = new DenseMatrix64F(6, 1);
      solver.solve(solution);
   }
}

