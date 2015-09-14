package us.ihmc.tools.inputDevices.ghostMouse;

import java.awt.event.KeyEvent;

public class GhostMouseStringToChar
{
   public static int convertStringToKeycode(String string)
   {
      if (string.equals("SHIFT")) return KeyEvent.VK_SHIFT;
      else if (string.equals("SPACE")) return KeyEvent.VK_SPACE;
      else if (string.equals("BACKSPACE")) return KeyEvent.VK_BACK_SPACE;

      else
      {
         int character = (int) (string.toUpperCase().charAt(0));
         return character;
      }
   }

   public static String convertKeycodeToString(int keyCode)
   {
      if (keyCode == KeyEvent.VK_SHIFT) return "SHIFT";
      else if (keyCode == KeyEvent.VK_SPACE) return "SPACE";
      else if (keyCode == KeyEvent.VK_BACK_SPACE) return "BACK_SPACE";
      
      else
      {
         char keyCharacter = (char) keyCode;
         String ret = String.valueOf(keyCharacter);
         ret = ret.toLowerCase();
         return ret;
      }
   }
}
