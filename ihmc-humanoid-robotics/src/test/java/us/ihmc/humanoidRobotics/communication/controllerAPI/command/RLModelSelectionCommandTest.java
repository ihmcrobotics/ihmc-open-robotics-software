package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.RLModelSelectionMessage;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.RLModelSelectionCommand;

import static org.junit.jupiter.api.Assertions.*;

public class RLModelSelectionCommandTest
{
   @Test
   public void testConverting()
   {
      int model = 5;
      RLModelSelectionMessage message = new RLModelSelectionMessage();
      message.setDesiredModel((byte) model);
      RLModelSelectionCommand command = new RLModelSelectionCommand();
      command.setFromMessage(message);
      assertEquals(model, command.getDesiredModel());
   }
}
