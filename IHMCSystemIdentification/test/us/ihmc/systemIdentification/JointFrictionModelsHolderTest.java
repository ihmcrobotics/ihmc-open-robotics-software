package us.ihmc.systemIdentification;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.systemIdentification.JointFrictionModelsHolder;
import us.ihmc.systemIdentification.frictionId.frictionModels.AsymmetricCoulombViscousFrictionModel;
import us.ihmc.systemIdentification.frictionId.frictionModels.AsymmetricCoulombViscousStribeckFrictionModel;
import us.ihmc.systemIdentification.frictionId.frictionModels.FrictionModel;
import us.ihmc.systemIdentification.frictionId.frictionModels.FrictionState;
import us.ihmc.systemIdentification.frictionId.frictionModels.JointFrictionModel;
import us.ihmc.systemIdentification.frictionId.frictionModels.NoCompensationFrictionModel;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class JointFrictionModelsHolderTest
{
   private static double epsilon = 1e-5;

   private double stictionTransitionVelocity = 0.011;
   private double smallVelocityAbs = JointFrictionModelsHolder.getSmallVelocityAbs();
   private double alphaForFilteredVelocity = 0.0;
   private double forceThreshold = 1.0;
   private double maxVelocityForCompensation = 5;

   private double positiveCoulomb = 5.5;
   private double positiveViscous = 0.2;
   private double negativeCoulomb = 6.1;
   private double negativeViscous = 0.21;

   private double positiveSigma = 220;
   private double positiveFc0 = 50;
   private double positiveFs0 = 30;
   private double positiveCs = 0.015;
   private double negativeSigma = 180;
   private double negativeFc0 = 50;
   private double negativeFs0 = 20;
   private double negativeCs = 0.007;

   private double requestedNonZeroForce = 10;
   private double requestedNonZeroForceUnderThreshold = 0.5;
   private double requestedZeroForce = 0.0;
   private double currentJointVelocityLessThanStictionVelocity = 0.9 * stictionTransitionVelocity;
   private double currentJointVelocityGreaterThanStictionVelocity = 1.1 * stictionTransitionVelocity;
   private double currentJointVelocityGreaterThanMaxVelocity = 5 * maxVelocityForCompensation;
   private double requestedNonZeroJointAcceleration = 1.2;
   private double requestedZeroJointAcceleration = 0.0;

   private double velocityForStictionInForceMode = smallVelocityAbs * Math.signum(requestedNonZeroForce);
   private double velocityForOutStictionInAccelerationMode = smallVelocityAbs * Math.signum(requestedNonZeroJointAcceleration);

   private NoCompensationFrictionModel noCompensatingModel = new NoCompensationFrictionModel();
   private AsymmetricCoulombViscousFrictionModel asymmetricCVModel = new AsymmetricCoulombViscousFrictionModel(positiveCoulomb,
         positiveViscous, negativeCoulomb, negativeViscous);
   private AsymmetricCoulombViscousStribeckFrictionModel asymmetricCVSModel = new AsymmetricCoulombViscousStribeckFrictionModel(positiveSigma, positiveFc0, positiveFs0, positiveCs,
                                                                                  negativeSigma, negativeFc0, negativeFs0, negativeCs);

   private String name = "simpleHolder";
   private YoVariableRegistry registry = new YoVariableRegistry("simpleRegistry");

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructorAndFrictionStateSelection()
   {
      JointFrictionModelsHolderForTest holder = new JointFrictionModelsHolderForTest(name, registry);

      // FrictionModel.OFF
      holder.setActiveFrictionModel(FrictionModel.OFF);
      FrictionModel model = holder.getActiveFrictionModel();
      assertEquals(FrictionModel.OFF, model);

      JointFrictionModel jointFrictionModel = holder.getActiveJointFrictionModel();
      assertEquals(noCompensatingModel, jointFrictionModel);

      holder.selectFrictionStateAndFrictionVelocity(requestedNonZeroForce, currentJointVelocityGreaterThanStictionVelocity, requestedNonZeroJointAcceleration);
      FrictionState state = holder.getCurrentFrictionState();
      double friction = holder.getCurrentFrictionForce();
      holder.resetVelocityForFrictionCalculation();
      assertEquals(FrictionState.NOT_COMPENSATING, state);
      assertEquals(0.0, friction, epsilon);

      // FrictionModel.ASYMMETRIC_COULOMB_VISCOUS or others
      holder.setActiveFrictionModel(FrictionModel.ASYMMETRIC_COULOMB_VISCOUS);
      FrictionModel model2 = holder.getActiveFrictionModel();
      holder.resetVelocityForFrictionCalculation();
      assertEquals(FrictionModel.ASYMMETRIC_COULOMB_VISCOUS, model2);

      JointFrictionModel jointFrictionModel2 = holder.getActiveJointFrictionModel();
      assertEquals(asymmetricCVModel, jointFrictionModel2);

      double velocity2 = holder.selectFrictionStateAndFrictionVelocity(requestedNonZeroForce, currentJointVelocityGreaterThanStictionVelocity,
            requestedNonZeroJointAcceleration);
      FrictionState state2 = holder.getCurrentFrictionState();
      holder.resetVelocityForFrictionCalculation();
      assertEquals(FrictionState.OUT_ST_ACCELERATION_D, state2);
      assertEquals(velocityForOutStictionInAccelerationMode, velocity2, epsilon);

      Double velocity3 = holder.selectFrictionStateAndFrictionVelocity(requestedZeroForce, currentJointVelocityGreaterThanStictionVelocity,
            requestedZeroJointAcceleration);
      FrictionState state3 = holder.getCurrentFrictionState();
      holder.resetVelocityForFrictionCalculation();
      assertEquals(FrictionState.NOT_COMPENSATING, state3);
      assertNull(velocity3);

      double velocity4 = holder.selectFrictionStateAndFrictionVelocity(requestedNonZeroForce, currentJointVelocityLessThanStictionVelocity,
            requestedZeroJointAcceleration);
      FrictionState state4 = holder.getCurrentFrictionState();
      holder.resetVelocityForFrictionCalculation();
      assertEquals(FrictionState.IN_ST_FORCE_D, state4);
      assertEquals(velocityForStictionInForceMode, velocity4, epsilon);

      double velocity5 = holder.selectFrictionStateAndFrictionVelocity(requestedZeroForce, currentJointVelocityLessThanStictionVelocity,
            requestedNonZeroJointAcceleration);
      FrictionState state5 = holder.getCurrentFrictionState();
      holder.resetVelocityForFrictionCalculation();
      assertEquals(FrictionState.IN_ST_ACCELERATION_D, state5);
      assertEquals(velocityForOutStictionInAccelerationMode, velocity5, epsilon);
    
      Double velocity6 = holder.selectFrictionStateAndFrictionVelocity(requestedNonZeroForceUnderThreshold, currentJointVelocityLessThanStictionVelocity,
            requestedZeroJointAcceleration);
      FrictionState state6 = holder.getCurrentFrictionState();
      double friction6 = holder.getCurrentFrictionForce();
      holder.resetVelocityForFrictionCalculation();
      assertEquals(FrictionState.NOT_COMPENSATING, state6);
      assertEquals(0.0, friction6, epsilon);
      assertNull(velocity6);
     
      Double velocity7 = holder.selectFrictionStateAndFrictionVelocity(requestedNonZeroForceUnderThreshold, currentJointVelocityGreaterThanMaxVelocity,
            requestedZeroJointAcceleration);
      FrictionState state7 = holder.getCurrentFrictionState();
      double friction7 = holder.getCurrentFrictionForce();
      holder.resetVelocityForFrictionCalculation();
      assertEquals(FrictionState.NOT_COMPENSATING, state7);
      assertEquals(0.0, friction7, epsilon);
      assertNull(velocity7);
   }

   private class JointFrictionModelsHolderForTest extends JointFrictionModelsHolder
   {
      public JointFrictionModelsHolderForTest(String name, YoVariableRegistry registry)
      {
         super(name, registry, alphaForFilteredVelocity, forceThreshold, stictionTransitionVelocity, maxVelocityForCompensation);
         frictionModels.put(FrictionModel.OFF, noCompensatingModel);
         frictionModels.put(FrictionModel.ASYMMETRIC_COULOMB_VISCOUS, asymmetricCVModel);
         frictionModels.put(FrictionModel.ASYMMETRIC_COULOMB_VISCOUS_STRIBECK, asymmetricCVSModel);
      }

      @Override
      protected void checkIfExistFrictionModelForThisJoint(FrictionModel requestedFrictionModel)
      {

      }
   }

}
