package us.ihmc.simulationconstructionset.joystick;

import net.java.games.input.Component;
import net.java.games.input.Event;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;

public class SystemOutJoystickEventListener implements JoystickEventListener
{
   public void processEvent(Event event)
   {
      StringBuffer buffer = new StringBuffer();
      buffer.append("at t = ");
      buffer.append(event.getNanos()).append(", ");
      Component comp = event.getComponent();
      buffer.append(comp.getName()).append(" changed to ");
      float value = event.getValue();
      if (comp.isAnalog())
      {
         buffer.append(value);
      }
      else
      {
         if (value == 1.0f)
         {
            buffer.append("On");
         }
         else
         {
            buffer.append("Off");
         }
      }

      System.out.println(buffer.toString());
   }
}