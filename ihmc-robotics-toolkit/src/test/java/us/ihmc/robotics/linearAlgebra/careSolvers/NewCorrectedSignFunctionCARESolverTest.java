package us.ihmc.robotics.linearAlgebra.careSolvers;

public class NewCorrectedSignFunctionCARESolverTest extends NewCARESolverTest
{
   @Override
   protected NewCARESolver getSolver()
   {
      return new NewDefectCorrectionCARESolver(new NewSignFunctionCARESolver());
   }

   @Override
   protected double getEpsilon()
   {
      return 1e-5;
   }
}
