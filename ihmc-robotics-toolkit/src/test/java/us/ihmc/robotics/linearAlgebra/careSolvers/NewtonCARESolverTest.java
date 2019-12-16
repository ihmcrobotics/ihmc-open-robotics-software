package us.ihmc.robotics.linearAlgebra.careSolvers;

public class NewtonCARESolverTest extends CARESolverTest
{
   @Override
   protected CARESolver getSolver()
   {
      return new NewtonCARESolver();
   }
}
