package us.ihmc.simulationconstructionset.joystick;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;
import us.ihmc.tools.inputDevices.joystick.JoystickComponentNotFoundException;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.io.printing.PrintTools;

public class JoystickUpdater implements Runnable, ComponentSelector
{
   private final Controller joystickController;
   private long pollIntervalMillis = 20;
   private final ArrayList<JoystickEventListener> listeners = new ArrayList<JoystickEventListener>();
   private HashSet<Identifier> identifiers = new HashSet<Identifier>();

   public JoystickUpdater()
   {
      this(findFirstStickController());
   }

   public JoystickUpdater(Controller joystickController)
   {
      this.joystickController = joystickController;

      for (Component component : joystickController.getComponents())
      {
         identifiers.add(component.getIdentifier());
      }
   }

   private static Controller findFirstStickController()
   {
      Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();

      List<Controller> sticks = new ArrayList<>(controllers.length);
      for (Controller controller : controllers)
      {
         if (controller.getType() == Controller.Type.STICK)
         {
            sticks.add(controller);
         }
      }

      if (sticks.size() > 1)
      {
         PrintTools.warn("More than one joystick found! " + sticks);
      }

      if (!sticks.isEmpty())
      {
         return sticks.get(0);
      }
      else
      {
         return null;
      }
   }

   public static boolean isJoyStickConnected()
   {
      return findFirstStickController() != null;
   }

   public Component findComponent(Identifier identifier) throws JoystickComponentNotFoundException
   {
      if (hasComponent(identifier))
         return joystickController.getComponent(identifier);
      else
         throw new JoystickComponentNotFoundException(identifier);
   }

   public boolean hasComponent(Identifier identifier)
   {
      return identifiers.contains(identifier);
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

   public void clearListeners()
   {
      this.listeners.clear();
   }
}