package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class DifferentialIDMassMatrixCalculatorTest extends MassMatrixCalculatorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testKineticEnergy()
   {
      setUpRandomChainRobot();
      double expectedKineticEnergy = computeKineticEnergy(joints);

      DifferentialIDMassMatrixCalculator massMatrixCalculator = new DifferentialIDMassMatrixCalculator(worldFrame, elevator);
      massMatrixCalculator.compute();
      DenseMatrix64F massMatrix = massMatrixCalculator.getMassMatrix();
      double kineticEnergyFromMassMatrix = computeKineticEnergy(joints, massMatrix);

      assertEquals(expectedKineticEnergy, kineticEnergyFromMassMatrix, 1e-12);
   }
}
