package us.ihmc.robotics.controllers.stiction;

import org.junit.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

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
}
