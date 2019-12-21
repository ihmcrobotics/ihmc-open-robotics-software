package us.ihmc.robotics.linearAlgebra.careSolvers;

public class HamiltonianIterativeSchurCARESolverTest extends CARESolverTest
{
   @Override
   protected CARESolver getSolver()
   {
      return new HamiltonianIterativeSchurCARESolver();
   }
}
