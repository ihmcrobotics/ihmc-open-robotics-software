package us.ihmc.robotics.linearAlgebra.careSolvers;

public class MatrixSignFunctionCARESolverTest extends CARESolverTest
{
   @Override
   protected CARESolver getSolver()
   {
      return new MatrixSignFunctionCARESolver();
   }
}
