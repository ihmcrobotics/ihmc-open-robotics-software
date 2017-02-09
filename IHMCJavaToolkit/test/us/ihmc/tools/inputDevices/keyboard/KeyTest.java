package us.ihmc.tools.inputDevices.keyboard;

import static org.junit.Assert.assertEquals;

import java.awt.event.KeyEvent;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class KeyTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testKey()
   {
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_LEFT), Key.LEFT);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_RIGHT), Key.RIGHT);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_UP), Key.UP);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_DOWN), Key.DOWN);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_SHIFT), Key.SHIFT);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_CONTROL), Key.CTRL);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_META), Key.META);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_ALT), Key.ALT);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_A), Key.A);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_B), Key.B);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_C), Key.C);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_D), Key.D);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_E), Key.E);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_F), Key.F);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_G), Key.G);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_H), Key.H);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_I), Key.I);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_J), Key.J);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_K), Key.K);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_L), Key.L);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_M), Key.M);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_N), Key.N);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_O), Key.O);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_P), Key.P);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_Q), Key.Q);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_R), Key.R);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_S), Key.S);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_T), Key.T);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_U), Key.U);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_V), Key.V);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_W), Key.W);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_X), Key.X);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_Y), Key.Y);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_Z), Key.Z);
      assertEquals("Key code not mapped", Key.fromKeyCode(KeyEvent.VK_DOLLAR), Key.UNDEFINED);
      
      assertEquals("From string no worky", Key.fromString("a"), Key.A);
      assertEquals("From string no worky", Key.fromString("$"), Key.UNDEFINED);
   }
}
