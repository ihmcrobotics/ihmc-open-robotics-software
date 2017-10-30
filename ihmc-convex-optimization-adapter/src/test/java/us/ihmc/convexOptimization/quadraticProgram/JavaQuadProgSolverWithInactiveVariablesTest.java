package us.ihmc.convexOptimization.quadraticProgram;

import org.junit.Ignore;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class JavaQuadProgSolverWithInactiveVariablesTest extends AbstractSimpleActiveSetQPSolverWithInactiveVariablesTest
{
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
      testSimpleCasesWithBoundsConstraints(0, 1, 6, true);
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
}
