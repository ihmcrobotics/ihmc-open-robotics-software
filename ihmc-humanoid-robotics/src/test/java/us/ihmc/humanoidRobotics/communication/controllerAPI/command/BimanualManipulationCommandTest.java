package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import controller_msgs.msg.dds.BimanualManipulationMessage;
import org.junit.jupiter.api.Test;

public class BimanualManipulationCommandTest
{
   @Test
   public void testClear()
   {
      BimanualManipulationCommand bimanualManipulationCommand = new BimanualManipulationCommand();
      bimanualManipulationCommand.clear();
      assertFalse(bimanualManipulationCommand.isDisableRequested());
      assertEquals(0.0, bimanualManipulationCommand.getObjectMass(), 1e-9);
      assertEquals(0.0, bimanualManipulationCommand.getSqueezeForce(), 1e-9);
      assertEquals(0.0, bimanualManipulationCommand.getInitializeDuration(), 1e-9);
      assertEquals(0.0, bimanualManipulationCommand.getTrackingErrorThreshold(), 1e-9);
   }

   @Test
   public void testIsCommandValid()
   {
      BimanualManipulationCommand bimanualManipulationCommand = new BimanualManipulationCommand();
      assertFalse(bimanualManipulationCommand.isCommandValid());

      bimanualManipulationCommand.setObjectMass(1.0);
      assertTrue(bimanualManipulationCommand.isCommandValid());

      bimanualManipulationCommand.setObjectMass(-1.0);
      assertFalse(bimanualManipulationCommand.isCommandValid());

      bimanualManipulationCommand.setObjectMass(1.0);
      bimanualManipulationCommand.setSqueezeForce(-1.0);
      assertFalse(bimanualManipulationCommand.isCommandValid());

      bimanualManipulationCommand.setSqueezeForce(1.0);
      assertTrue(bimanualManipulationCommand.isCommandValid());
   }

   @Test
   public void testBimanualManipulationCommand()
   {
      new BimanualManipulationCommand();
   }

   @Test
   public void testSet()
   {
      BimanualManipulationCommand command = new BimanualManipulationCommand();
      command.setDisable(true);
      command.setObjectMass(1.0);
      command.setSqueezeForce(2.0);
      command.setInitializeDuration(3.0);
      command.setTrackingErrorThreshold(4.0);
      assertTrue(command.isDisableRequested());
      assertEquals(1.0, command.getObjectMass(), 1e-9);
      assertEquals(2.0, command.getSqueezeForce(), 1e-9);
      assertEquals(3.0, command.getInitializeDuration(), 1e-9);
      assertEquals(4.0, command.getTrackingErrorThreshold(), 1e-9);
   }

   @Test
   public void testSetFromMessage()
   {
      BimanualManipulationCommand bimanualManipulationCommand = new BimanualManipulationCommand();
      BimanualManipulationMessage message = new BimanualManipulationMessage();
      message.setDisable(true);
      message.setObjectMass(1.0);
      message.setSqueezeForce(2.0);
      message.setInitializeDuration(3.0);
      message.setAcceptableTrackingError(4.0);
      bimanualManipulationCommand.setFromMessage(message);
      assertTrue(bimanualManipulationCommand.isDisableRequested());
      assertEquals(1.0, bimanualManipulationCommand.getObjectMass(), 1e-9);
      assertEquals(2.0, bimanualManipulationCommand.getSqueezeForce(), 1e-9);
      assertEquals(3.0, bimanualManipulationCommand.getInitializeDuration(), 1e-9);
      assertEquals(4.0, bimanualManipulationCommand.getTrackingErrorThreshold(), 1e-9);
   }

   @Test
   public void testSetFromCommand()
   {
      BimanualManipulationCommand bimanualManipulationCommand = new BimanualManipulationCommand();
      BimanualManipulationCommand other = new BimanualManipulationCommand();
      other.setDisable(true);
      other.setObjectMass(1.0);
      other.setSqueezeForce(2.0);
      other.setInitializeDuration(3.0);
      other.setTrackingErrorThreshold(4.0);
      bimanualManipulationCommand.set(other);
      assertTrue(bimanualManipulationCommand.isDisableRequested());
      assertEquals(1.0, bimanualManipulationCommand.getObjectMass(), 1e-9);
      assertEquals(2.0, bimanualManipulationCommand.getSqueezeForce(), 1e-9);
      assertEquals(3.0, bimanualManipulationCommand.getInitializeDuration(), 1e-9);
      assertEquals(4.0, bimanualManipulationCommand.getTrackingErrorThreshold(), 1e-9);
   }

   @Test
   public void testGetMessageClass()
   {
      assertEquals(BimanualManipulationMessage.class, new BimanualManipulationCommand().getMessageClass());
   }
}
