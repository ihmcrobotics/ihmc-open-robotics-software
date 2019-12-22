package us.ihmc.robotics.linearAlgebra.careSolvers.matrixSignFunction;

public class NewtonMatrixSignFunctionTest extends MatrixSignFunctionTest
{
   @Override
   protected MatrixSignFunction getSolver()
   {
      return new NewtonMatrixSignFunction();
   }

}
