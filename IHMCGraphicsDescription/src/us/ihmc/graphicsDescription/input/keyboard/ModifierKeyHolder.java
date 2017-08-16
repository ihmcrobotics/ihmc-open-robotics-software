package us.ihmc.graphicsDescription.input.keyboard;

import java.util.EnumMap;

import us.ihmc.tools.inputDevices.keyboard.Key;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

public class ModifierKeyHolder implements ModifierKeyInterface
{
   private final EnumMap<Key, Boolean> pressedModifierKeys = new EnumMap<Key, Boolean>(Key.class);
 
   @Override
   public boolean isKeyPressed(Key key)
   {
      Boolean isPressed = pressedModifierKeys.get(key);
      
      if(isPressed == null)
      {
         return false;
      }
      else
      {
         return isPressed;
      }
   }
   
   public void setKeyState(Key key, boolean state)
   {
      pressedModifierKeys.put(key, state);
   }
}
