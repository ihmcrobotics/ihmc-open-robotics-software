package us.ihmc.convexOptimization.quadraticProgram;

import org.junit.Ignore;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.exceptions.NoConvergenceException;

import static org.junit.Assert.assertEquals;

public class JavaQuadProgSolverWithInactiveVariablesTest extends AbstractSimpleActiveSetQPSolverWithInactiveVariablesTest
{
   private static final double epsilon = 1e-4;

   @Override
   public ActiveSetQPSolverWithInactiveVariablesInterface createSolverToTest()
   {
      JavaQuadProgSolverWithInactiveVariables solver = new JavaQuadProgSolverWithInactiveVariables();
      solver.setUseWarmStart(false);
      return solver;
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleCasesWithInequalityConstraintsAndInactiveVariables()
   {
      testSimpleCasesWithInequalityConstraintsAndInactiveVariables(0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000000)
   public void testSimpleCasesWithBoundsConstraintsAndInactiveVariables()
   {
      testSimpleCasesWithBoundsConstraintsAndInactiveVariables(0, 1, 2, 0, false);
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
