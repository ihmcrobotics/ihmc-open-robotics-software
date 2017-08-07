package us.ihmc.systemIdentification.frictionId.frictionModels;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class PressureBasedFrictionModelTest
{
   private static double epsilon = 1e-5;

   private double x1 = 1.5;
   private double x2 = 0.2;
   private double x3 = 10.5;
   private double x4 = 5.1;
   private double x5 = 0.9;

   private double positiveVelocity = 1.2;
   private double negativeVelocity = -1.5;
   private double zeroVelocity = 0.0;
   private double posPressure = 123;
   private double negPressure = 105;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructorAndFormula()
   {
      PressureBasedFrictionModel model = new PressureBasedFrictionModel(x1, x2, x3, x4, x5);

      FrictionModel frictionModel = model.getFrictionModel();
      assertEquals(FrictionModel.PRESSURE_BASED, frictionModel);

      model.computeFrictionForce(positiveVelocity, negPressure, posPressure);
      double friction = model.getFrictionForce();
      assertEquals(727.4868737254822, friction, epsilon);

      model.computeFrictionForce(zeroVelocity, negPressure, posPressure);
      double friction2 = model.getFrictionForce();
      assertEquals(726.0, friction2, epsilon);

      model.computeFrictionForce(negativeVelocity, negPressure, posPressure);
      double friction3 = model.getFrictionForce();
      assertEquals(724.2612273310225, friction3, epsilon);
   }

}
