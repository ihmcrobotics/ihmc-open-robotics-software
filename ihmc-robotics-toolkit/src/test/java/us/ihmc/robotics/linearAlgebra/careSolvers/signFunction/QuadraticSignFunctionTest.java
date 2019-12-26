package us.ihmc.robotics.linearAlgebra.careSolvers.signFunction;

public class QuadraticSignFunctionTest extends SignFunctionTest
{
   @Override
   protected SignFunction getSolver()
   {
      return new QuadraticSignFunction();
   }

}
