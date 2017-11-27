package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public abstract class NullspaceProjectorCalculatorTest
{
   public abstract NullspaceProjectorCalculator getNullspaceProjectorCalculator();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleNullspaceProjector()
   {
      NullspaceProjectorCalculator nullspaceProjectorCalculator = getNullspaceProjectorCalculator();

      DenseMatrix64F jacobian = new DenseMatrix64F(2, 2);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 1, 9.0);

      DenseMatrix64F nullspaceProjector = new DenseMatrix64F(2, 2);
      nullspaceProjectorCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      for (int i = 0; i < nullspaceProjector.getNumElements(); i++)
         assertEquals(nullspaceProjector.get(i), 0.0, 1e-7);


      jacobian = new DenseMatrix64F(2, 4);
      jacobian.set(0, 0, 1.0);
      jacobian.set(0, 1, 3.0);
      jacobian.set(0, 3, 7.0);
      jacobian.set(1, 0, 7.0);
      jacobian.set(1, 2, 9.0);
      jacobian.set(1, 3, 11.0);

      nullspaceProjector = new DenseMatrix64F(4, 4);
      nullspaceProjectorCalculator.computeNullspaceProjector(jacobian, nullspaceProjector);

      assertEquals(nullspaceProjector.get(0, 0), 0.746421, 1e-4);
      assertEquals(nullspaceProjector.get(0, 1), 0.130401, 1e-4);
      assertEquals(nullspaceProjector.get(0, 2),-0.381917, 1e-4);
      assertEquals(nullspaceProjector.get(0, 3),-0.162518, 1e-4);

      assertEquals(nullspaceProjector.get(1, 0), 0.130401, 1e-4);
      assertEquals(nullspaceProjector.get(1, 1), 0.708629, 1e-4);
      assertEquals(nullspaceProjector.get(1, 2), 0.292532, 1e-4);
      assertEquals(nullspaceProjector.get(1, 3),-0.322327, 1e-4);

      assertEquals(nullspaceProjector.get(2, 0),-0.381917, 1e-4);
      assertEquals(nullspaceProjector.get(2, 1), 0.292532, 1e-4);
      assertEquals(nullspaceProjector.get(2, 2), 0.383593, 1e-4);
      assertEquals(nullspaceProjector.get(2, 3),-0.0708113, 1e-4);

      assertEquals(nullspaceProjector.get(3, 0),-0.162518, 1e-4);
      assertEquals(nullspaceProjector.get(3, 1),-0.322327, 1e-4);
      assertEquals(nullspaceProjector.get(3, 2),-0.0708113, 1e-4);
      assertEquals(nullspaceProjector.get(3, 3), 0.161357, 1e-4);
   }
}
