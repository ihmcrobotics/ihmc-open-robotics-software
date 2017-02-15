package us.ihmc.systemIdentification.frictionId.frictionModels;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class AsymmetricCoulombViscousStribeckFrictionModelTest
{
   private static double epsilon = 1e-5;

   private double positiveSigma = 220;
   private double positiveFc0 = 50;
   private double positiveFs0 = 30;
   private double positiveCs = 0.015;
   private double negativeSigma = 180;
   private double negativeFc0 = 50;
   private double negativeFs0 = 20;
   private double negativeCs = 0.007;

   private double positiveInStribeckVelocity = 0.01;
   private double negativeInStribeckVelocity = -0.002;
   private double positiveOutStribeckVelocity = 0.2;
   private double negativeOutStribeckVelocity = -0.18;
   private double zeroVelocity = 0.0;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructorAndFormula()
   {
      AsymmetricCoulombViscousStribeckFrictionModel model = new AsymmetricCoulombViscousStribeckFrictionModel(positiveSigma,
            positiveFc0, positiveFs0, positiveCs, negativeSigma, negativeFc0, negativeFs0, negativeCs);

      FrictionModel frictionModel = model.getFrictionModel();
      assertEquals(FrictionModel.ASYMMETRIC_COULOMB_VISCOUS_STRIBECK, frictionModel);

      model.computeFrictionForce(positiveInStribeckVelocity);
      double friction = model.getFrictionForce();
      assertEquals(67.60251357097776, friction, epsilon);

      model.computeFrictionForce(negativeInStribeckVelocity);
      double friction2 = model.getFrictionForce();
      assertEquals(-65.38954586150572, friction2, epsilon);

      model.computeFrictionForce(positiveOutStribeckVelocity);
      double friction3 = model.getFrictionForce();
      assertEquals(94.00004858790376, friction3, epsilon);

      model.computeFrictionForce(negativeOutStribeckVelocity);
      double friction4 = model.getFrictionForce();
      assertEquals(-82.40000000013598, friction4, epsilon);

      model.computeFrictionForce(zeroVelocity);
      double friction5 = model.getFrictionForce();
      assertEquals(0.0, friction5, epsilon);

   }
}
