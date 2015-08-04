package us.ihmc.FrictionID.frictionModels;

import org.junit.Test;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;

import static org.junit.Assert.assertEquals;

public class NoCompensationFrictionModelTest
{
   private static double epsilon = 1e-10;

   private double positiveVelocity = 1.2;
   private double negativeVelocity = -1.5;
   private double zeroVelocity = 0.0;

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testConstructorAndFormula()
   {
      NoCompensationFrictionModel model = new NoCompensationFrictionModel();

      FrictionModel frictionModel = model.getFrictionModel();
      assertEquals(FrictionModel.OFF, frictionModel);

      model.computeFrictionForce(positiveVelocity);
      double friction = model.getFrictionForce();
      assertEquals(0.0, friction, epsilon);

      model.computeFrictionForce(zeroVelocity);
      double friction2 = model.getFrictionForce();
      assertEquals(0.0, friction2, epsilon);

      model.computeFrictionForce(negativeVelocity);
      double friction3 = model.getFrictionForce();
      assertEquals(0.0, friction3, epsilon);
   }
}
