package us.ihmc.tools.inputDevices.joystick;

import java.util.ArrayList;

import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import us.ihmc.tools.io.printing.PrintTools;

public class Joystick
{
   private final ArrayList<JoystickEventListener> listeners = new ArrayList<JoystickEventListener>();
   private final ArrayList<JoystickGeneralListener> generalListenersList = new ArrayList<JoystickGeneralListener>();

   public Joystick()
   {
      Controller joystickController = getFirstJoystickFoundOnSystem();

      if (joystickController == null)
      {
         PrintTools.error("Joystick not found!");
         return;
      }
      
      JoystickUpdater joystickUpdater = new JoystickUpdater(joystickController, listeners, generalListenersList);

      Thread thread = new Thread(joystickUpdater);
      thread.setPriority(Thread.NORM_PRIORITY);
      thread.start();
   }

   private static Controller getFirstJoystickFoundOnSystem()
   {
      Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();

      ArrayList<Controller> joystickControllers = new ArrayList<Controller>();
      for (Controller controller : controllers)
      {
         if (controller.getType() == Controller.Type.STICK)
         {
            joystickControllers.add(controller);
         }
      }

      if (joystickControllers.size() > 1)
      {
         PrintTools.warn("More than one joystick found! " + joystickControllers);
      }

      if (!joystickControllers.isEmpty())
      {
         PrintTools.info("Using joystick: " + joystickControllers.get(0));
         return joystickControllers.get(0);
      }
      else
      {
         return null;
      }
   }

   public void addJoystickEventListener(JoystickEventListener joystickEventListener)
   {
      listeners.add(joystickEventListener);
   }

   public void addJoystickGeneralListener(JoystickGeneralListener joystickGeneralListener)
   {
      generalListenersList.add(joystickGeneralListener);
   }
   
   public static boolean isJoyStickConnected()
   {
      return getFirstJoystickFoundOnSystem() != null;
   }

   public static void main(String[] args)
   {
      try
      {
         new Joystick();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
