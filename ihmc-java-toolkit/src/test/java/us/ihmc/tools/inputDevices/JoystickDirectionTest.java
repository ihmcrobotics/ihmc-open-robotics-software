package us.ihmc.tools.inputDevices;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class JoystickDirectionTest
{

	@Test
   public void testGetFromJoystickPOV()
   {
      assertEquals(JoystickDirection.getFromJoystickPOV(-0.1f), null);
      assertEquals(JoystickDirection.getFromJoystickPOV(360.5f), null);


      assertEquals(JoystickDirection.getFromJoystickPOV(340.0f), JoystickDirection.NORTH);
      assertEquals(JoystickDirection.getFromJoystickPOV(22.1f), JoystickDirection.NORTH);

      assertEquals(JoystickDirection.getFromJoystickPOV(23.2f), JoystickDirection.NORTH_EAST);
      assertEquals(JoystickDirection.getFromJoystickPOV(65.0f), JoystickDirection.NORTH_EAST);

      assertEquals(JoystickDirection.getFromJoystickPOV(68.1f), JoystickDirection.EAST);
      assertEquals(JoystickDirection.getFromJoystickPOV(112.3f), JoystickDirection.EAST);

      assertEquals(JoystickDirection.getFromJoystickPOV(113.2f), JoystickDirection.SOUTH_EAST);
      assertEquals(JoystickDirection.getFromJoystickPOV(150.6f), JoystickDirection.SOUTH_EAST);

      assertEquals(JoystickDirection.getFromJoystickPOV(160.1f), JoystickDirection.SOUTH);
      assertEquals(JoystickDirection.getFromJoystickPOV(200.5f), JoystickDirection.SOUTH);

      assertEquals(JoystickDirection.getFromJoystickPOV(204.5f), JoystickDirection.SOUTH_WEST);
      assertEquals(JoystickDirection.getFromJoystickPOV(240.5f), JoystickDirection.SOUTH_WEST);

      assertEquals(JoystickDirection.getFromJoystickPOV(250.1f), JoystickDirection.WEST);
      assertEquals(JoystickDirection.getFromJoystickPOV(291.9f), JoystickDirection.WEST);

      assertEquals(JoystickDirection.getFromJoystickPOV(295.6f), JoystickDirection.NORTH_WEST);
      assertEquals(JoystickDirection.getFromJoystickPOV(330.5f), JoystickDirection.NORTH_WEST);


   }

}
