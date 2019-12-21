package us.ihmc.robotics.linearAlgebra.careSolvers;

public class HamiltonianDirectSchurCARESolverTest extends CARESolverTest
{
   @Override
   protected CARESolver getSolver()
   {
      return new HamiltonianDirectSchurCARESolver();
   }
}
