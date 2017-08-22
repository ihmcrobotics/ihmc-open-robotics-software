package us.ihmc.graphicsDescription.input.mouse;

import java.awt.event.InputEvent;

public enum MouseButton
{
   LEFT, RIGHT, MIDDLE, LEFTRIGHT;
   
   public static final MouseButton[] values = values();
   
   public int getInputEventMask()
   {
      if (equals(LEFT))
         return InputEvent.BUTTON1_DOWN_MASK;
      if (equals(MIDDLE))
         return InputEvent.BUTTON2_DOWN_MASK;
      if (equals(RIGHT))
         return InputEvent.BUTTON3_DOWN_MASK;
      else
         return -1;
   }
   
   public String toShortString()
   {
      if (equals(LEFT))
         return "L";
      if (equals(MIDDLE))
         return "M";
      if (equals(RIGHT))
         return "R";
      else
         return "LR";
   }
}
