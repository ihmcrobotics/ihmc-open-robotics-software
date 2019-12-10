package us.ihmc.robotics.linearAlgebra.careSolvers;

public class HamiltonianCARESolverTest extends CARESolverTest
{
   @Override
   protected CARESolver getSolver()
   {
      return new HamiltonianCARESolver();
   }
}
