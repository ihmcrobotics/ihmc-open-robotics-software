package us.ihmc.tools.inputDevices.mouse;

import static org.junit.Assert.assertTrue;

import java.awt.event.InputEvent;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class MouseButtonTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMouseButton()
   {
      assertTrue("Valu not rite", MouseButton.LEFT.getInputEventMask() == InputEvent.BUTTON1_DOWN_MASK);
      assertTrue("Valu not rite", MouseButton.MIDDLE.getInputEventMask() == InputEvent.BUTTON2_DOWN_MASK);
      assertTrue("Valu not rite", MouseButton.RIGHT.getInputEventMask() == InputEvent.BUTTON3_DOWN_MASK);
      assertTrue("Valu not rite", MouseButton.LEFTRIGHT.getInputEventMask() == -1);
      
      assertTrue("Valu not rite", MouseButton.LEFT.toShortString().equals("L"));
      assertTrue("Valu not rite", MouseButton.MIDDLE.toShortString().equals("M"));
      assertTrue("Valu not rite", MouseButton.RIGHT.toShortString().equals("R"));
      assertTrue("Valu not rite", MouseButton.LEFTRIGHT.toShortString().equals("LR"));
   }
}
