package us.ihmc.robotics.controllers.stiction;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.robotics.controllers.stiction.StictionCompensator.StictionActionMode;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.parameters.XmlParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

import static us.ihmc.robotics.controllers.stiction.StictionCompensator.StictionActionMode.Moving;

public class StictionCompensatorTest
{
   private static final int iters = 100;
   private static final double epsilon = 1e-8;
   private static final double controlDT = 1e-2;

   @Test
   public void testSetDesiredTorque()
   {
      YoRegistry registry = new YoRegistry("test");
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

   @Test
   public void testResetStictionCompensation()
   {
      YoRegistry registry = new YoRegistry("test");
      YoDouble constantStictionProvider = new YoDouble("constantStictionProvider", registry);
      StictionModel stictionModel = new ConstantStictionModel(constantStictionProvider);

      StictionCompensator stictionCompensator = new StictionCompensator("", stictionModel, controlDT, registry);

      YoDouble stictionCompensationLimit = (YoDouble) registry.findVariable("_StictionCompensationLimit");
      YoDouble stictionCompensation = (YoDouble) registry.findVariable("_StictionCompensation");

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

   @Test
   public void testComputeStictionCompensation()
   {
      YoRegistry registry = new YoRegistry("test");
      YoDouble constantStictionProvider = new YoDouble("constantStictionProvider", registry);
      StictionModel stictionModel = new ConstantStictionModel(constantStictionProvider);

      StictionCompensator stictionCompensator = new StictionCompensator("", stictionModel, controlDT, registry);

      YoDouble stictionCompensationLimit = (YoDouble) registry.findVariable("_StictionCompensationLimit");
      YoDouble yoStictionCompensation = (YoDouble) registry.findVariable("_StictionCompensation");

      YoDouble stictionCompensationRate = (YoDouble) registry.findVariable("_StictionCompensationRate");
      YoDouble desiredTorqueStictionLimitFactor = (YoDouble) registry.findVariable("_DesiredTorqueStictionLimitFactor");
      YoDouble minTimeInMode = (YoDouble) registry.findVariable("_MinTimeInMode");
      YoEnum<StictionActionMode> actionMode = (YoEnum<StictionActionMode>) registry.findVariable("_StictionActionMode");


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
         assertEquals(stictionCompensation, stictionCompensator.getStictionCompensation(), epsilon);

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

   @Test
   public void testActionForwardMode()
   {
      YoRegistry registry = new YoRegistry("test");
      YoDouble constantStictionProvider = new YoDouble("constantStictionProvider", registry);
      StictionModel stictionModel = new ConstantStictionModel(constantStictionProvider);

      StictionCompensator stictionCompensator = new StictionCompensator("", stictionModel, controlDT, registry);

      YoDouble minTimeInMode = (YoDouble) registry.findVariable("_MinTimeInMode");

      YoEnum<StictionActionMode> actionMode = (YoEnum<StictionActionMode>) registry.findVariable("_StictionActionMode");

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

   @Test
   public void testActionBackwardMode()
   {
      YoRegistry registry = new YoRegistry("test");
      YoDouble constantStictionProvider = new YoDouble("constantStictionProvider", registry);
      StictionModel stictionModel = new ConstantStictionModel(constantStictionProvider);

      StictionCompensator stictionCompensator = new StictionCompensator("", stictionModel, controlDT, registry);

      YoDouble minTimeInMode = (YoDouble) registry.findVariable("_MinTimeInMode");

      YoEnum<StictionActionMode> actionMode = (YoEnum<StictionActionMode>) registry.findVariable("_StictionActionMode");

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
