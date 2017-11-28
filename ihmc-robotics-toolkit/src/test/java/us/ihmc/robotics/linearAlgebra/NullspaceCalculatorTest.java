package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import static org.junit.Assert.assertEquals;

public abstract class NullspaceCalculatorTest
{
   public abstract NullspaceCalculator getNullspaceProjectorCalculator();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleNullspaceProjector()
   {
      NullspaceCalculator nullspaceCalculator = getNullspaceProjectorCalculator();

      DenseMatrix64F jacobian = new DenseMatrix64F(2, 2);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 1, 9.0);

      DenseMatrix64F nullspaceProjector = new DenseMatrix64F(2, 2);
      /*
      nullspaceCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      for (int i = 0; i < nullspaceProjector.getNumElements(); i++)
         assertEquals(nullspaceProjector.get(i), 0.0, 1e-7);
         */


      jacobian = new DenseMatrix64F(2, 4);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(0, 3, 7.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 2, 9.0);
      jacobian.set(1, 3, 11.0);

      nullspaceProjector = new DenseMatrix64F(4, 4);
      nullspaceCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      DenseMatrix64F nullspaceProjectorExpected = new DenseMatrix64F(4, 4);

      nullspaceProjectorExpected.set(0, 0, 0.746421);
      nullspaceProjectorExpected.set(0, 1, 0.130401);
      nullspaceProjectorExpected.set(0, 2,-0.381917);
      nullspaceProjectorExpected.set(0, 3,-0.162518);

      nullspaceProjectorExpected.set(1, 0, 0.130401);
      nullspaceProjectorExpected.set(1, 1, 0.708629);
      nullspaceProjectorExpected.set(1, 2, 0.292532);
      nullspaceProjectorExpected.set(1, 3,-0.322327);

      nullspaceProjectorExpected.set(2, 0,-0.381917);
      nullspaceProjectorExpected.set(2, 1, 0.292532);
      nullspaceProjectorExpected.set(2, 2, 0.383593);
      nullspaceProjectorExpected.set(2, 3,-0.0708113);

      nullspaceProjectorExpected.set(3, 0,-0.162518);
      nullspaceProjectorExpected.set(3, 1,-0.322327);
      nullspaceProjectorExpected.set(3, 2,-0.0708113);
      nullspaceProjectorExpected.set(3, 3, 0.161357);

      for (int i = 0; i < nullspaceProjectorExpected.getNumElements(); i++)
         assertEquals(nullspaceProjectorExpected.get(i), nullspaceProjector.get(i), 1e-5);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleProjectOntoNullspace()
   {
      NullspaceCalculator nullspaceCalculator = getNullspaceProjectorCalculator();

      DenseMatrix64F jacobian = new DenseMatrix64F(2, 2);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 1, 9.0);

      DenseMatrix64F vectorToProject = new DenseMatrix64F(1, 2);
      vectorToProject.set(0, 0, 3.5);
      vectorToProject.set(0, 1, 4.5);

      DenseMatrix64F projectedVector = new DenseMatrix64F(1, 2);

      nullspaceCalculator.projectOntoNullspace(vectorToProject, jacobian, projectedVector);

      assertEquals(0.0, projectedVector.get(0, 0), 1e-7);
      assertEquals(0.0, projectedVector.get(0, 1), 1e-7);


      jacobian = new DenseMatrix64F(2, 4);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(0, 3, 7.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 2, 9.0);
      jacobian.set(1, 3, 11.0);

      DenseMatrix64F matrixToProject = new DenseMatrix64F(2, 4);
      matrixToProject.set(0, 0, 3.5);
      matrixToProject.set(0, 1, 4.5);
      matrixToProject.set(0, 2, 5.5);
      matrixToProject.set(0, 3, 6.5);
      matrixToProject.set(1, 0, 7.5);
      matrixToProject.set(1, 1, 8.5);
      matrixToProject.set(1, 2, 9.5);
      matrixToProject.set(1, 3, 10.5);

      projectedVector = new DenseMatrix64F(2, 4);

      nullspaceCalculator.projectOntoNullspace(matrixToProject, jacobian, projectedVector);

      double epsilon = 1e-5;
      assertEquals(0.0423707, projectedVector.get(0, 0), epsilon);
      assertEquals(3.15904, projectedVector.get(0, 1), epsilon);
      assertEquals(1.62918, projectedVector.get(0, 2), epsilon);
      assertEquals(-1.35993, projectedVector.get(0, 3), epsilon);

      assertEquals(1.37192, projectedVector.get(1, 0), epsilon);
      assertEquals(6.39598, projectedVector.get(1, 1), epsilon);
      assertEquals(2.52277, projectedVector.get(1, 2), epsilon);
      assertEquals(-2.93712, projectedVector.get(1, 3), epsilon);
   }
}
