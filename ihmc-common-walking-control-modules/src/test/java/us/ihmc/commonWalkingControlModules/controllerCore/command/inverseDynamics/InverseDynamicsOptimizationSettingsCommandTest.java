package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;

class InverseDynamicsOptimizationSettingsCommandTest
{
   @Test
   void testHasField() throws Exception
   {
      Random random = new Random(567654743);
      InverseDynamicsOptimizationSettingsCommand command = new InverseDynamicsOptimizationSettingsCommand();
      assertFalse(command.hasRhoMin());
      assertFalse(command.hasJointAccelerationMax());
      assertFalse(command.hasRhoWeight());
      assertFalse(command.hasRhoRateWeight());
      assertFalse(command.hasCenterOfPressureWeight());
      assertFalse(command.hasCenterOfPressureRateWeight());
      assertFalse(command.hasJointAccelerationWeight());
      assertFalse(command.hasJointJerkWeight());
      assertFalse(command.hasJointTorqueWeight());

      command.setRhoMin(random.nextDouble());
      assertTrue(command.hasRhoMin());
      command.setJointAccelerationMax(random.nextDouble());
      assertTrue(command.hasJointAccelerationMax());
      command.setRhoWeight(random.nextDouble());
      assertTrue(command.hasRhoWeight());
      command.setRhoRateWeight(random.nextDouble());
      assertTrue(command.hasRhoRateWeight());
      command.setCenterOfPressureWeight(EuclidCoreRandomTools.nextPoint2D(random));
      assertTrue(command.hasCenterOfPressureWeight());
      command.setCenterOfPressureRateWeight(EuclidCoreRandomTools.nextPoint2D(random));
      assertTrue(command.hasCenterOfPressureRateWeight());
      command.setJointAccelerationWeight(random.nextDouble());
      assertTrue(command.hasJointAccelerationWeight());
      command.setJointJerkWeight(random.nextDouble());
      assertTrue(command.hasJointJerkWeight());
      command.setJointTorqueWeight(random.nextDouble());
      assertTrue(command.hasJointTorqueWeight());
   }

   @Test
   void testGetter() throws Exception
   {
      Random random = new Random(567654743);
      InverseDynamicsOptimizationSettingsCommand command = new InverseDynamicsOptimizationSettingsCommand();

      double rhoMin = random.nextDouble();
      double jointAccelerationMax = random.nextDouble();
      double rhoWeight = random.nextDouble();
      double rhoRateWeight = random.nextDouble();
      Point2D centerOfPressureWeight = EuclidCoreRandomTools.nextPoint2D(random);
      Point2D centerOfPressureRateWeight = EuclidCoreRandomTools.nextPoint2D(random);
      double jointAccelerationWeight = random.nextDouble();
      double jointJerkWeight = random.nextDouble();
      double jointTorqueWeight = random.nextDouble();

      command.setRhoMin(rhoMin);
      assertEquals(rhoMin, command.getRhoMin());
      command.setJointAccelerationMax(jointAccelerationMax);
      assertEquals(jointAccelerationMax, command.getJointAccelerationMax());
      command.setRhoWeight(rhoWeight);
      assertEquals(rhoWeight, command.getRhoWeight());
      command.setRhoRateWeight(rhoRateWeight);
      assertEquals(rhoRateWeight, command.getRhoRateWeight());
      command.setCenterOfPressureWeight(centerOfPressureWeight);
      assertEquals(centerOfPressureWeight, command.getCenterOfPressureWeight());
      command.setCenterOfPressureRateWeight(centerOfPressureRateWeight);
      assertEquals(centerOfPressureRateWeight, command.getCenterOfPressureRateWeight());
      command.setJointAccelerationWeight(jointAccelerationWeight);
      assertEquals(jointAccelerationWeight, command.getJointAccelerationWeight());
      command.setJointJerkWeight(jointJerkWeight);
      assertEquals(jointJerkWeight, command.getJointJerkWeight());
      command.setJointTorqueWeight(jointTorqueWeight);
      assertEquals(jointTorqueWeight, command.getJointTorqueWeight());
   }
}
