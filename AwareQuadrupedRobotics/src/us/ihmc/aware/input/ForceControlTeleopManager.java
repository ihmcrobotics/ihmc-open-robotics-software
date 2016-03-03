package us.ihmc.aware.input;

import java.util.HashMap;
import java.util.Map;

public class ForceControlTeleopManager implements MotionEventCallback
{
   private final XboxControllerInputDevice input = new XboxControllerInputDevice();
   private final Map<InputDeviceAxis, Double> axisValues = new HashMap();

   public ForceControlTeleopManager()
   {
      input.registerCallback(this);
   }

   public void start()
   {
      input.poll();
   }

   @Override
   public void onMotionEvent(MotionEvent e)
   {
      System.out.println("Event: " + e.getAxis() + " = " + e.getValue());
      axisValues.put(e.getAxis(), e.getValue());

      switch (e.getAxis())
      {
      case HOME_BUTTON:
         break;
      case VIEW_BUTTON:
         break;
      case MENU_BUTTON:
         break;
      case LEFT_BUTTON:
         break;
      case RIGHT_BUTTON:
         break;
      case LEFT_TRIGGER:
      case RIGHT_TRIGGER:
         break;
      case LEFT_STICK_X:
      case LEFT_STICK_Y:
         break;
      case RIGHT_STICK_X:
      case RIGHT_STICK_Y:
         break;
      case BUTTON_A:
         break;
      case BUTTON_B:
         break;
      case BUTTON_X:
         break;
      case BUTTON_Y:
         break;
      case D_PAD_UP:
         break;
      case D_PAD_RIGHT:
         break;
      case D_PAD_DOWN:
         break;
      case D_PAD_LEFT:
         break;
      }
   }

   public static void main(String[] args)
   {
      new ForceControlTeleopManager().start();
   }
}
