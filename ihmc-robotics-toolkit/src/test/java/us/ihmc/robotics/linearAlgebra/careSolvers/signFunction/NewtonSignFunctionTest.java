package us.ihmc.robotics.linearAlgebra.careSolvers.signFunction;

public class NewtonSignFunctionTest extends SignFunctionTest
{
   @Override
   protected SignFunction getSolver()
   {
      return new NewtonSignFunction();
   }

}
