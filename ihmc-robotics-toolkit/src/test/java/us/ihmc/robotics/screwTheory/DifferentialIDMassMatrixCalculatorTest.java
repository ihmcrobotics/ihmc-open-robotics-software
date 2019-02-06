package us.ihmc.robotics.screwTheory;

import static us.ihmc.robotics.Assert.*;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class DifferentialIDMassMatrixCalculatorTest extends MassMatrixCalculatorTest
{

	@Test
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
