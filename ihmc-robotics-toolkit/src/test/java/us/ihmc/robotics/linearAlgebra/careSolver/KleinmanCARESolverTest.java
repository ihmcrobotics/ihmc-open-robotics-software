package us.ihmc.robotics.linearAlgebra.careSolver;

import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.KleinmanCARESolver;

public class KleinmanCARESolverTest extends CARESolverTest
{
   @Override
   protected CARESolver getSolver()
   {
      return new KleinmanCARESolver();
   }
}
