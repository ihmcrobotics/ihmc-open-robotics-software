package us.ihmc.tools.inputDevices.joystick;

import net.java.games.input.Event;
import us.ihmc.commons.PrintTools;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;

public class JoystickDemo
{
   public JoystickDemo()
   {
      try
      {
         Joystick joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
         joystick.setStandalone();
         joystick.printListOfConnectedJoysticks();
         joystick.addJoystickStatusListener(new JoystickStatusListener()
         {
            @Override
            public void updateConnectivity(boolean connected)
            {
               PrintTools.info("Connected: " + connected);
            }
         });
         joystick.addJoystickEventListener(new JoystickEventListener()
         {
            @Override
            public void processEvent(Event event)
            {
               PrintTools.info("Event: " + event.toString());
            }
         });
         joystick.setPollInterval(10);
      }
      catch (JoystickNotFoundException e)
      {
         e.printStackTrace();
      }
   }
   
   public static void main(String[] args)
   {
      new JoystickDemo();
   }
}
