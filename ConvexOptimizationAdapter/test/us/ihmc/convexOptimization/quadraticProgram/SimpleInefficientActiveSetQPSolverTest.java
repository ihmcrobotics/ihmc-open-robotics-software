package us.ihmc.convexOptimization.quadraticProgram;

public class SimpleInefficientActiveSetQPSolverTest extends AbstractSimpleActiveSetQPSolverTest
{
   @Override
   public SimpleActiveSetQPSolverInterface createSolverToTest()
   {
      return new SimpleInefficientActiveSetQPSolver();
   }
}
