package us.ihmc.robotics.linearAlgebra.careSolvers.matrixSignFunction;

public class QuadraticMatrixSignFunctionTest extends MatrixSignFunctionTest
{
   @Override
   protected MatrixSignFunction getSolver()
   {
      return new QuadraticMatrixSignFunction();
   }

}
