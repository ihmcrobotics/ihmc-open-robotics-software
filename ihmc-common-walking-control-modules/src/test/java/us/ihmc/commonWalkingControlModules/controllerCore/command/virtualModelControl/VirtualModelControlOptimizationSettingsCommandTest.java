package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;

class VirtualModelControlOptimizationSettingsCommandTest
{
   @Test
   void testHasField() throws Exception
   {
      Random random = new Random(567654743);
      VirtualModelControlOptimizationSettingsCommand command = new VirtualModelControlOptimizationSettingsCommand();
      assertFalse(command.hasRhoMin());
      assertFalse(command.hasRhoWeight());
      assertFalse(command.hasRhoRateWeight());
      assertFalse(command.hasCenterOfPressureWeight());
      assertFalse(command.hasCenterOfPressureRateWeight());
      assertFalse(command.hasMomentumRateWeight());
      assertFalse(command.hasMomentumAccelerationWeight());

      command.setRhoMin(random.nextDouble());
      assertTrue(command.hasRhoMin());
      command.setRhoWeight(random.nextDouble());
      assertTrue(command.hasRhoWeight());
      command.setRhoRateWeight(random.nextDouble());
      assertTrue(command.hasRhoRateWeight());
      command.setCenterOfPressureWeight(EuclidCoreRandomTools.nextPoint2D(random));
      assertTrue(command.hasCenterOfPressureWeight());
      command.setCenterOfPressureRateWeight(EuclidCoreRandomTools.nextPoint2D(random));
      assertTrue(command.hasCenterOfPressureRateWeight());
      command.setMomentumRateWeight(random.nextDouble());
      assertTrue(command.hasMomentumRateWeight());
      command.setMomentumAccelerationWeight(random.nextDouble());
      assertTrue(command.hasMomentumAccelerationWeight());
   }

   @Test
   void testGetter() throws Exception
   {
      Random random = new Random(567654743);
      VirtualModelControlOptimizationSettingsCommand command = new VirtualModelControlOptimizationSettingsCommand();

      double rhoMin = random.nextDouble();
      double rhoWeight = random.nextDouble();
      double rhoRateWeight = random.nextDouble();
      Point2D centerOfPressureWeight = EuclidCoreRandomTools.nextPoint2D(random);
      Point2D centerOfPressureRateWeight = EuclidCoreRandomTools.nextPoint2D(random);
      double momentumRateWeight = random.nextDouble();
      double momentumAccelerationWeight = random.nextDouble();

      command.setRhoMin(rhoMin);
      assertEquals(rhoMin, command.getRhoMin());
      command.setRhoWeight(rhoWeight);
      assertEquals(rhoWeight, command.getRhoWeight());
      command.setRhoRateWeight(rhoRateWeight);
      assertEquals(rhoRateWeight, command.getRhoRateWeight());
      command.setCenterOfPressureWeight(centerOfPressureWeight);
      assertEquals(centerOfPressureWeight, command.getCenterOfPressureWeight());
      command.setCenterOfPressureRateWeight(centerOfPressureRateWeight);
      assertEquals(centerOfPressureRateWeight, command.getCenterOfPressureRateWeight());
      command.setMomentumRateWeight(momentumRateWeight);
      assertEquals(momentumRateWeight, command.getMomentumRateWeight());
      command.setMomentumAccelerationWeight(momentumAccelerationWeight);
      assertEquals(momentumAccelerationWeight, command.getMomentumAccelerationWeight());
   }
}
