package us.ihmc.convexOptimization.quadraticProgram;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SimpleDiagonalActiveSetQPSolverTest extends AbstractSimpleActiveSetQPSolverTest
{
   @Override
   public SimpleActiveSetQPSolverInterface createSolverToTest()
   {
      return new SimpleDiagonalActiveSetQPSolver();
   }
}
