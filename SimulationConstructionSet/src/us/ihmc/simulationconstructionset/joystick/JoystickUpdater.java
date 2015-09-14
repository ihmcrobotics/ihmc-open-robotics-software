package us.ihmc.simulationconstructionset.joystick;

import java.util.ArrayList;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import us.ihmc.tools.inputDevices.JInputLibraryLoader;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Event;
import net.java.games.input.EventQueue;

public class JoystickUpdater implements Runnable
{
   private final Controller joystickController;
   private long pollIntervalMillis = 20;
   private final ArrayList<JoystickEventListener> listeners = new ArrayList<JoystickEventListener>();

   public JoystickUpdater()
   {
      this(findFirstController());
   }

   
   public JoystickUpdater(Controller joystickController)
   {
      this.joystickController = joystickController;
   }
   
   
   /**
    * Just return the first stick you find
    */
   private static Controller findFirstController()
   {
      JInputLibraryLoader.loadLibraries();
      Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();

      for (Controller controller : controllers)
      {
         if (controller.getType() == Controller.Type.STICK)
         {
            return controller;
         }
      }
      throw new JoyStickNotFoundException();

   }
   
   public static boolean isJoyStickConnected()
   {
      try{
              findFirstController();
              return true;
      }
      catch(JoyStickNotFoundException e)
      {
         return false;
      }
      
   }

   
   public void listComponents()
   {
      for(Component component: joystickController.getComponents())
      {
         System.out.println(component.getIdentifier().getName());
      }
   }
   
   public Component findComponent(Identifier identifier)
   {
      for (Component component : joystickController.getComponents())
      {
         if (component.getIdentifier().equals(identifier))
            return component;
      }
      
      throw new RuntimeException("component with identifier " + identifier + " not found");
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