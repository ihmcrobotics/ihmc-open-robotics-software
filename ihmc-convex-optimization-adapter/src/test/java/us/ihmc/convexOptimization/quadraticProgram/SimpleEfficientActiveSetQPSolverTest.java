package us.ihmc.convexOptimization.quadraticProgram;

public class SimpleEfficientActiveSetQPSolverTest extends AbstractSimpleActiveSetQPSolverTest
{
   @Override
   public SimpleActiveSetQPSolverInterface createSolverToTest()
   {
      SimpleEfficientActiveSetQPSolver simpleEfficientActiveSetQPSolver = new SimpleEfficientActiveSetQPSolver();
      simpleEfficientActiveSetQPSolver.setUseWarmStart(false);
      return simpleEfficientActiveSetQPSolver;
   }
}
