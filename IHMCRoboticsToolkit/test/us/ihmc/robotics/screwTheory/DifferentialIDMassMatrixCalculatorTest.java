package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

import static org.junit.Assert.assertEquals;

public class DifferentialIDMassMatrixCalculatorTest extends MassMatrixCalculatorTest
{

	@EstimatedDuration(duration = 0.0)
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
