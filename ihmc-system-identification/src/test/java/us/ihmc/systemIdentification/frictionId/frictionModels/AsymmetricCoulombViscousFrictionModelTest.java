package us.ihmc.systemIdentification.frictionId.frictionModels;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class AsymmetricCoulombViscousFrictionModelTest
{
   private static double epsilon = 1e-10;

   private double positiveCoulomb = 5.5;
   private double positiveViscous = 0.2;
   private double negativeCoulomb = 6.1;
   private double negativeViscous = 0.21;

   private double positiveVelocity = 1.2;
   private double negativeVelocity = -1.5;
   private double zeroVelocity = 0.0;

	@Test
   public void testConstructorAndFormula()
   {
      AsymmetricCoulombViscousFrictionModel model = new AsymmetricCoulombViscousFrictionModel(positiveCoulomb, positiveViscous,
            negativeCoulomb, negativeViscous);

      FrictionModel frictionModel = model.getFrictionModel();
      assertEquals(FrictionModel.ASYMMETRIC_COULOMB_VISCOUS, frictionModel);

      model.computeFrictionForce(positiveVelocity);
      double friction = model.getFrictionForce();
      assertEquals(5.74, friction, epsilon);

      model.computeFrictionForce(zeroVelocity);
      double friction2 = model.getFrictionForce();
      assertEquals(0.0, friction2, epsilon);

      model.computeFrictionForce(negativeVelocity);
      double friction3 = model.getFrictionForce();
      assertEquals(-6.415, friction3, epsilon);
   }
}
