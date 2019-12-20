package us.ihmc.robotics.linearAlgebra.careSolvers;

public class HamiltonianEigenCARESolverTest extends CARESolverTest
{
   @Override
   protected CARESolver getSolver()
   {
      return new HamiltonianEigenCARESolver();
   }
}
