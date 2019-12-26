package us.ihmc.robotics.linearAlgebra.careSolvers;

public class NewEigenvectorCARESolverTest extends NewCARESolverTest
{
   @Override
   protected NewCARESolver getSolver()
   {
      return new NewEigenvectorCARESolver();
   }

   @Override
   protected double getEpsilon()
   {
      return 1e-4;
   }
}
