package us.ihmc.tools.inputDevices.keyboard;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.graphicsDescription.input.keyboard.ModifierKeyHolder;
import us.ihmc.tools.inputDevices.keyboard.Key;

public class ModifierKeyHolderTest
{
   @Test
   public void testModifierKeyHolderHoldsKeys()
   {
      ModifierKeyHolder modifierKeyHolder = new ModifierKeyHolder();
      modifierKeyHolder.setKeyState(Key.CTRL, true);
      modifierKeyHolder.setKeyState(Key.ALT, false);
      
      assertTrue("Key holder has questionable morals", modifierKeyHolder.isKeyPressed(Key.CTRL));
      assertFalse("Key holder has questionable morals", modifierKeyHolder.isKeyPressed(Key.ALT));
      assertFalse("Key holder has questionable morals", modifierKeyHolder.isKeyPressed(Key.SHIFT));
   }
}
