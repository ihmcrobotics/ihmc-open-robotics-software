package us.ihmc.tools.inputDevices.mouse;

import static org.junit.Assert.*;

import java.awt.event.InputEvent;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class MouseButtonTest
{
   @DeployableTestMethod(estimatedDuration = 0.1)
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
