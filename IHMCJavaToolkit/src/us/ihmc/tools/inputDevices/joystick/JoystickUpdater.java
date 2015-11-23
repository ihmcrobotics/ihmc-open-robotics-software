package us.ihmc.tools.inputDevices.joystick;

import java.util.ArrayList;
import java.util.HashMap;

import net.java.games.input.Controller;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;

public class JoystickUpdater implements Runnable
{
   private static final boolean DEBUG = false;
   private final Controller joystickController;
   private HashMap<String, Float> lastValues = new HashMap<String, Float>();
   
   private final ArrayList<JoystickEventListener> listeners;
   private final ArrayList<JoystickGeneralListener> generalListenersList;
   
   private final int pollIntervalMillis = 20;
   private float deadband = 0.2f;
   private boolean connected;

   public JoystickUpdater(Controller joystickController, ArrayList<JoystickEventListener> listeners, ArrayList<JoystickGeneralListener> generalListenersList)
   {
      this.joystickController = joystickController;
      this.listeners = listeners;
      this.generalListenersList = generalListenersList;
      
      connected = true;
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