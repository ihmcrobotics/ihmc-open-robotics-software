package us.ihmc.robotics.linearAlgebra.careSolvers;

public class HamiltonianSchurCARESolverTest extends CARESolverTest
{
   @Override
   protected CARESolver getSolver()
   {
      return new HamiltonianSchurCARESolver();
   }
}
