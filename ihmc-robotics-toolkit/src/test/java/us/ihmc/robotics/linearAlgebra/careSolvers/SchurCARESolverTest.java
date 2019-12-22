package us.ihmc.robotics.linearAlgebra.careSolvers;

public class SchurCARESolverTest extends CARESolverTest
{
   @Override
   protected CARESolver getSolver()
   {
      return new SchurCARESolver();
   }
}
