package us.ihmc.mecano;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

public class DifferentialIDMassMatrixCalculatorTest extends MassMatrixCalculatorTest
{

	@Test
   public void testKineticEnergy()
   {
      setUpRandomChainRobot();
      double expectedKineticEnergy = computeKineticEnergy(joints);

      DifferentialIDMassMatrixCalculator massMatrixCalculator = new DifferentialIDMassMatrixCalculator(worldFrame, elevator);
      massMatrixCalculator.compute();
      DMatrixRMaj massMatrix = massMatrixCalculator.getMassMatrix();
      double kineticEnergyFromMassMatrix = computeKineticEnergy(joints, massMatrix);

      assertEquals(expectedKineticEnergy, kineticEnergyFromMassMatrix, 1e-12);
   }
}
