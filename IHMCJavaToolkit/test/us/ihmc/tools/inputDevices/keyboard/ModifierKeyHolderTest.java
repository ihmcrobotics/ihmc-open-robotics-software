package us.ihmc.tools.inputDevices.keyboard;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ModifierKeyHolderTest
{
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
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
