package us.ihmc.simulationconstructionset.joystick;

import java.util.ArrayList;

import net.java.games.input.Controller;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;

public class JoystickUpdater implements Runnable
{
   private final Controller joystickController;
   private long pollIntervalMillis = 20;
   private final ArrayList<JoystickEventListener> listeners = new ArrayList<JoystickEventListener>();

   public JoystickUpdater(Controller joystickController)
   {
      this.joystickController = joystickController;
   }

   public void setPollInterval(long pollIntervalMillis)
   {
      this.pollIntervalMillis = pollIntervalMillis;
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
            for (JoystickEventListener listener : listeners)
            {
               listener.processEvent(event);
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

   public void addListener(JoystickEventListener joystickEventListener)
   {
      this.listeners.add(joystickEventListener);
   }
}