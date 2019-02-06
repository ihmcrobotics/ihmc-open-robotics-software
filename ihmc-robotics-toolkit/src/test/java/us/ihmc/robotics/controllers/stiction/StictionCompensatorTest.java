package us.ihmc.robotics.controllers.stiction;

import org.junit.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.controllers.stiction.StictionCompensator.StictionActionMode;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.parameters.XmlParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

import static us.ihmc.robotics.controllers.stiction.StictionCompensator.StictionActionMode.Moving;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class StictionCompensatorTest
{
   private static final int iters = 100;
   private static final double epsilon = 1e-8;
   private static final double controlDT = 1e-2;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetDesiredTorque()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble constantStictionProvider = new YoDouble("constantStictionProvider", registry);
      StictionModel stictionModel = new ConstantStictionModel(constantStictionProvider);

      StictionCompensator stictionCompensator = new StictionCompensator("", stictionModel, controlDT, registry);

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double desiredTorqueExpected = RandomNumbers.nextDouble(random, -1000.0, 1000.0);
         stictionCompensator.setDesiredTorque(desiredTorqueExpected);

         assertEquals(desiredTorqueExpected, stictionCompensator.getDesiredTorque(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testResetStictionCompensation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble constantStictionProvider = new YoDouble("constantStictionProvider", registry);
      StictionModel stictionModel = new ConstantStictionModel(constantStictionProvider);

      StictionCompensator stictionCompensator = new StictionCompensator("", stictionModel, controlDT, registry);

      YoDouble stictionCompensationLimit = (YoDouble) registry.getVariable("_StictionCompensationLimit");
      YoDouble stictionCompensation = (YoDouble) registry.getVariable("_StictionCompensation");

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         stictionCompensationLimit.set(RandomNumbers.nextDouble(random, 1000.0));
         stictionCompensation.set(RandomNumbers.nextDouble(random, 1000.0));

         assertNotEquals(0.0, stictionCompensationLimit.getDoubleValue(), epsilon);
         assertNotEquals(0.0, stictionCompensation.getDoubleValue(), epsilon);

         stictionCompensator.resetStictionCompensation();

         assertEquals(0.0, stictionCompensationLimit.getDoubleValue(), epsilon);
         assertEquals(0.0, stictionCompensation.getDoubleValue(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeStictionCompensation()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble constantStictionProvider = new YoDouble("constantStictionProvider", registry);
      StictionModel stictionModel = new ConstantStictionModel(constantStictionProvider);

      StictionCompensator stictionCompensator = new StictionCompensator("", stictionModel, controlDT, registry);

      YoDouble stictionCompensationLimit = (YoDouble) registry.getVariable("_StictionCompensationLimit");
      YoDouble yoStictionCompensation = (YoDouble) registry.getVariable("_StictionCompensation");

      YoDouble stictionCompensationRate = (YoDouble) registry.getVariable("_StictionCompensationRate");
      YoDouble desiredTorqueStictionLimitFactor = (YoDouble) registry.getVariable("_DesiredTorqueStictionLimitFactor");
      YoDouble minTimeInMode = (YoDouble) registry.getVariable("_MinTimeInMode");
      YoEnum<StictionActionMode> actionMode = (YoEnum<StictionActionMode>) registry.getVariable("_StictionActionMode");


      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(registry);

      // have to call up front to introduce the rate limiting
      stictionCompensator.computeStictionCompensation();


      Random random = new Random(1738L);
      double expectedStictionCompensation = 0.0;
      for (int iter = 0; iter < iters; iter++)
      {
         double stictionMagnitude = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double limitFactor = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double expectedDesiredTorque = RandomNumbers.nextDouble(random, 50.0);
         double maxRate = RandomNumbers.nextDouble(random, 0.0, 100.0);

         constantStictionProvider.set(stictionMagnitude);
         desiredTorqueStictionLimitFactor.set(limitFactor);
         stictionCompensationRate.set(maxRate);

         stictionCompensator.setDesiredTorque(expectedDesiredTorque);
         stictionCompensator.setDesiredAcceleration(5.0 * Math.signum(expectedDesiredTorque));
         stictionCompensator.setVelocities(5.0 * Math.signum(expectedDesiredTorque), 5.0 * Math.signum(expectedDesiredTorque));

         actionMode.set(Moving);
         double stictionCompensation = stictionCompensator.computeStictionCompensation();

         assertEquals(stictionCompensation, yoStictionCompensation.getDoubleValue(), epsilon);
         assertEquals(stictionCompensation, stictionCompensator.getStictionCompensation(), expectedDesiredTorque);

         double torqueSign = Math.signum(expectedDesiredTorque);
         double maxCompensationFromTorque = torqueSign * limitFactor * expectedDesiredTorque;
         double expectedCompensationLimit = Math.min(maxCompensationFromTorque, stictionMagnitude);

         assertEquals(expectedCompensationLimit, stictionCompensationLimit.getDoubleValue(), epsilon);

         double expectedCompensation = torqueSign * expectedCompensationLimit;
         double expectedCompensationLowerLimit = expectedStictionCompensation - maxRate * controlDT;
         double expectedCompensationUpperLimit = expectedStictionCompensation + maxRate * controlDT;
         expectedStictionCompensation = MathTools.clamp(expectedCompensation, expectedCompensationLowerLimit, expectedCompensationUpperLimit);

         assertEquals("Iter " + iter, expectedStictionCompensation, stictionCompensation, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testActionForwardMode()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble constantStictionProvider = new YoDouble("constantStictionProvider", registry);
      StictionModel stictionModel = new ConstantStictionModel(constantStictionProvider);

      StictionCompensator stictionCompensator = new StictionCompensator("", stictionModel, controlDT, registry);

      YoDouble minTimeInMode = (YoDouble) registry.getVariable("_MinTimeInMode");

      YoEnum<StictionActionMode> actionMode = (YoEnum<StictionActionMode>) registry.getVariable("_StictionActionMode");

      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(registry);

      StictionActionMode currentExpectedMode = actionMode.getEnumValue();

      stictionCompensator.setVelocities(1.0, 1.0);
      stictionCompensator.setDesiredAcceleration(5.0);
      stictionCompensator.setDesiredTorque(5.0);

      // mode shouldn't change
      for (int i = 0; i < minTimeInMode.getDoubleValue() / controlDT ; i++)
      {
         stictionCompensator.computeStictionCompensation();
         assertEquals(currentExpectedMode, actionMode.getEnumValue());
      }

      stictionCompensator.computeStictionCompensation();

      currentExpectedMode = StictionActionMode.Accelerating;
      assertEquals(currentExpectedMode, actionMode.getEnumValue());

      stictionCompensator.setDesiredAcceleration(0.0);
      // mode shouldn't change
      for (int i = 0; i < minTimeInMode.getDoubleValue() / controlDT ; i++)
      {
         stictionCompensator.computeStictionCompensation();
         assertEquals(currentExpectedMode, actionMode.getEnumValue());
      }

      stictionCompensator.computeStictionCompensation();

      currentExpectedMode = Moving;
      assertEquals(currentExpectedMode, actionMode.getEnumValue());

      stictionCompensator.setDesiredAcceleration(-5.0);
      // mode shouldn't change
      for (int i = 0; i < minTimeInMode.getDoubleValue() / controlDT ; i++)
      {
         stictionCompensator.computeStictionCompensation();
         assertEquals(currentExpectedMode, actionMode.getEnumValue());
      }

      stictionCompensator.computeStictionCompensation();

      currentExpectedMode = StictionActionMode.Braking;
      assertEquals(currentExpectedMode, actionMode.getEnumValue());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testActionBackwardMode()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble constantStictionProvider = new YoDouble("constantStictionProvider", registry);
      StictionModel stictionModel = new ConstantStictionModel(constantStictionProvider);

      StictionCompensator stictionCompensator = new StictionCompensator("", stictionModel, controlDT, registry);

      YoDouble minTimeInMode = (YoDouble) registry.getVariable("_MinTimeInMode");

      YoEnum<StictionActionMode> actionMode = (YoEnum<StictionActionMode>) registry.getVariable("_StictionActionMode");

      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(registry);

      StictionActionMode currentExpectedMode = actionMode.getEnumValue();

      stictionCompensator.setVelocities(-1.0, -1.0);
      stictionCompensator.setDesiredAcceleration(-5.0);
      stictionCompensator.setDesiredTorque(-5.0);

      // mode shouldn't change
      for (int i = 0; i < minTimeInMode.getDoubleValue() / controlDT ; i++)
      {
         stictionCompensator.computeStictionCompensation();
         assertEquals(currentExpectedMode, actionMode.getEnumValue());
      }

      stictionCompensator.computeStictionCompensation();

      currentExpectedMode = StictionActionMode.Accelerating;
      assertEquals(currentExpectedMode, actionMode.getEnumValue());

      stictionCompensator.setDesiredAcceleration(0.0);
      // mode shouldn't change
      for (int i = 0; i < minTimeInMode.getDoubleValue() / controlDT ; i++)
      {
         stictionCompensator.computeStictionCompensation();
         assertEquals(currentExpectedMode, actionMode.getEnumValue());
      }

      stictionCompensator.computeStictionCompensation();

      currentExpectedMode = Moving;
      assertEquals(currentExpectedMode, actionMode.getEnumValue());

      stictionCompensator.setDesiredAcceleration(5.0);
      // mode shouldn't change
      for (int i = 0; i < minTimeInMode.getDoubleValue() / controlDT ; i++)
      {
         stictionCompensator.computeStictionCompensation();
         assertEquals(currentExpectedMode, actionMode.getEnumValue());
      }

      stictionCompensator.computeStictionCompensation();

      currentExpectedMode = StictionActionMode.Braking;
      assertEquals(currentExpectedMode, actionMode.getEnumValue());
   }
}
