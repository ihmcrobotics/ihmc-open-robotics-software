package us.ihmc.tools.inputDevices.joystick;

import java.util.ArrayList;
import java.util.HashMap;

import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;

public class Joystick
{
   private boolean DEBUG = false;
   private final int pollIntervalMillis = 20;
   private float deadband = 0.2f;
   private boolean connected = false;

   private final ArrayList<JoystickEventListener> listeners = new ArrayList<JoystickEventListener>();
   private final ArrayList<JoystickGeneralListener> generalListenersList = new ArrayList<JoystickGeneralListener>();

   public Joystick() throws Exception
   {
      Controller joystickController = getFirstJoystickFoundOnSystem();

      if (joystickController == null)
      {
         connected = false;

         throw new Exception("joystick not found");
      }
      else
      {
         connected = true;
      }

      JoystickUpdater joystickUpdater = new JoystickUpdater(joystickController);

      Thread thread = new Thread(joystickUpdater);
      thread.setPriority(Thread.NORM_PRIORITY);
      thread.start();
   }

   private Controller getFirstJoystickFoundOnSystem()
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

      if (DEBUG)
      {
         System.out.println("Found " + joystickControllers.size() + " joystick controllers");

         for (Controller controller : joystickControllers)
         {
            System.out.println("\t" + controller);
         }
      }

      if (joystickControllers.size() > 0)
      {
         Controller selectedController = joystickControllers.get(0);
         if (DEBUG)
         {
            System.out.println("using " + selectedController.getName());
         }

         return selectedController;
      }

      return null;
   }

   public void addJoystickEventListener(JoystickEventListener joystickEventListener)
   {
      listeners.add(joystickEventListener);
   }

   public void addJoystickGeneralListener(JoystickGeneralListener joystickGeneralListener)
   {
      generalListenersList.add(joystickGeneralListener);
   }

   public class JoystickUpdater implements Runnable
   {
      private final Controller joystickController;
      HashMap<String, Float> lastValues = new HashMap<String, Float>();

      public JoystickUpdater(Controller joystickController)
      {
         this.joystickController = joystickController;
      }

      public void run()
      {
         while (true)
         {
            joystickController.poll();
            EventQueue queue = joystickController.getEventQueue();
            Event event = new Event();

            while (queue.getNextEvent(event))
            {
               if (isJoystickAxisEvent(event))
               {
                  if (isInDeadBand(event))
                  {
                     event.set(event.getComponent(), 0.0f, event.getNanos());
                  }
                  else
                  {
                     event.set(event.getComponent(), scaleValue(event), event.getNanos());
                  }
               }

               if (isNewValue(event))
               {
                  if (DEBUG)
                  {
                     System.out.println("event = " + event);
                  }

                  for (JoystickEventListener listener : listeners)
                  {
                     listener.processEvent(event);

                  }

                  for (JoystickGeneralListener listener : generalListenersList)
                  {
                     listener.updateConnectivity(connected);

                  }
               }
            }

            try
            {
               Thread.sleep(pollIntervalMillis);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }
      }

      private boolean isNewValue(Event event)
      {
         Float value = lastValues.get(event.getComponent().getName());
         if (value != null)
         {
            if (event.getValue() == value)
            {
               lastValues.put(event.getComponent().getName(), value);

               return false;
            }
         }

         lastValues.put(event.getComponent().getName(), event.getValue());

         return true;
      }

      private float scaleValue(Event event)
      {
         if (event.getValue() > 0.0f)
         {
            return (event.getValue() - deadband) / (1.0f - deadband);
         }
         else
         {
            return (event.getValue() + deadband) / (1.0f - deadband);
         }
      }

      private boolean isInDeadBand(Event event)
      {
         return (event.getValue() < deadband) && (event.getValue() > -deadband);
      }

      private boolean isJoystickAxisEvent(Event event)
      {
         String name = event.getComponent().getName();

         return name.equals("X Axis") || name.equals("Y Axis") || name.equals("Z Rotation");
      }
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
