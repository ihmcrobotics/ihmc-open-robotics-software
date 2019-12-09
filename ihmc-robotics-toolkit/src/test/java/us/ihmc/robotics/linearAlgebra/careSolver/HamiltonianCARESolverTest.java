package us.ihmc.robotics.linearAlgebra.careSolver;

import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.HamiltonianCARESolver;

public class HamiltonianCARESolverTest extends CARESolverTest
{
   @Override
   protected CARESolver getSolver()
   {
      return new HamiltonianCARESolver();
   }
}
