package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

class InverseKinematicsOptimizationSettingsCommandTest
{
   @Test
   void testHasField() throws Exception
   {
      Random random = new Random(567654743);
      InverseKinematicsOptimizationSettingsCommand command = new InverseKinematicsOptimizationSettingsCommand();
      assertFalse(command.hasJointVelocityWeight());
      assertFalse(command.hasJointAccelerationWeight());

      command.setJointVelocityWeight(random.nextDouble());
      assertTrue(command.hasJointVelocityWeight());
      command.setJointAccelerationWeight(random.nextDouble());
      assertTrue(command.hasJointAccelerationWeight());
   }

   @Test
   void testGetter() throws Exception
   {
      Random random = new Random(567654743);
      InverseKinematicsOptimizationSettingsCommand command = new InverseKinematicsOptimizationSettingsCommand();

      double jointVelocityWeight = random.nextDouble();
      double jointAccelerationWeight = random.nextDouble();

      command.setJointVelocityWeight(jointVelocityWeight);
      assertEquals(jointVelocityWeight, command.getJointVelocityWeight());
      command.setJointAccelerationWeight(jointAccelerationWeight);
      assertEquals(jointAccelerationWeight, command.getJointAccelerationWeight());
   }
}
