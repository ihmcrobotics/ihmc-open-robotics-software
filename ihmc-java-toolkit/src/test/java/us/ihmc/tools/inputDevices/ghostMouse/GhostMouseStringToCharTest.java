package us.ihmc.tools.inputDevices.ghostMouse;

import static us.ihmc.robotics.Assert.*;

import java.awt.event.KeyEvent;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class GhostMouseStringToCharTest
{

	@Test
   public void testConvertStringToKeycode()
   {
      assertEquals(KeyEvent.VK_SPACE, GhostMouseStringToChar.convertStringToKeycode("SPACE"));
      assertEquals(KeyEvent.VK_SHIFT, GhostMouseStringToChar.convertStringToKeycode("SHIFT"));
      assertEquals(KeyEvent.VK_A, GhostMouseStringToChar.convertStringToKeycode("a"));
      assertEquals(KeyEvent.VK_B, GhostMouseStringToChar.convertStringToKeycode("b"));
      assertEquals(KeyEvent.VK_F, GhostMouseStringToChar.convertStringToKeycode("f"));
      assertEquals(KeyEvent.VK_Z, GhostMouseStringToChar.convertStringToKeycode("z"));
      assertEquals(KeyEvent.VK_PERIOD, GhostMouseStringToChar.convertStringToKeycode("."));
      assertEquals(KeyEvent.VK_0, GhostMouseStringToChar.convertStringToKeycode("0"));
      assertEquals(KeyEvent.VK_1, GhostMouseStringToChar.convertStringToKeycode("1"));
      assertEquals(KeyEvent.VK_9, GhostMouseStringToChar.convertStringToKeycode("9"));
   }

	@Test
   public void testConvertKeycodeToString()
   {
      assertEquals("SPACE", GhostMouseStringToChar.convertKeycodeToString(KeyEvent.VK_SPACE));
      assertEquals("SHIFT", GhostMouseStringToChar.convertKeycodeToString(KeyEvent.VK_SHIFT));
      assertEquals("a", GhostMouseStringToChar.convertKeycodeToString(KeyEvent.VK_A));
      assertEquals("q", GhostMouseStringToChar.convertKeycodeToString(KeyEvent.VK_Q));
      assertEquals("r", GhostMouseStringToChar.convertKeycodeToString(KeyEvent.VK_R));
      assertEquals(".", GhostMouseStringToChar.convertKeycodeToString(KeyEvent.VK_PERIOD));
      assertEquals("0", GhostMouseStringToChar.convertKeycodeToString(KeyEvent.VK_0));
      assertEquals("1", GhostMouseStringToChar.convertKeycodeToString(KeyEvent.VK_1));
      assertEquals("9", GhostMouseStringToChar.convertKeycodeToString(KeyEvent.VK_9));
   }
   
}
