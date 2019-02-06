package us.ihmc.tools.inputDevices.mouse;

import static us.ihmc.robotics.Assert.*;

import java.awt.event.InputEvent;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.graphicsDescription.input.mouse.MouseButton;

public class MouseButtonTest
{
   @Test
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
