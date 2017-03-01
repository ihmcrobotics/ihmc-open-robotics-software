package us.ihmc.convexOptimization.quadraticProgram;

public class SimpleDiagonalActiveSetQPSolverTest extends AbstractSimpleActiveSetQPSolverTest
{
   @Override
   public SimpleActiveSetQPSolverInterface createSolverToTest()
   {
      return new SimpleEfficientActiveSetQPSolver();
   }
}
