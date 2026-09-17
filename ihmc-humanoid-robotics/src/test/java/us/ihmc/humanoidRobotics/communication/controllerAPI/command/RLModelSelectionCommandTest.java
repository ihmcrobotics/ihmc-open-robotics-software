package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import static org.junit.jupiter.api.Assertions.*;

import controller_msgs.RLModelSelectionMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.RLModelSelectionCommand;

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
