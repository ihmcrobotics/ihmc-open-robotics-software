package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.MimicManeuverCommandMessage;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class MimicManeuverCommandTest
{
   @Test
   public void testConverting()
   {
      MimicManeuverCommandMessage message = new MimicManeuverCommandMessage();
      message.setRequestedAction(MimicManeuverCommandMessage.ACTION_STAND_UP);
      message.setExecute(true);

      MimicManeuverCommand command = new MimicManeuverCommand();
      command.setFromMessage(message);

      assertEquals(MimicManeuverCommandMessage.ACTION_STAND_UP, command.getRequestedAction());
      assertTrue(command.getExecute());
   }
}
