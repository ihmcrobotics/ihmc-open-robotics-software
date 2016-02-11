package us.ihmc.tools.inputDevices.joystick;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;

import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Component.Identifier;
import us.ihmc.tools.io.printing.PrintTools;

public class Joystick
{
   private final ArrayList<JoystickStatusListener> statusListeners = new ArrayList<JoystickStatusListener>();
   private final HashSet<Identifier> identifiers = new HashSet<Identifier>();
   private final Controller joystickController;
   private final JoystickUpdater joystickUpdater;
   private final JoystickModel model;

   public Joystick() throws IOException
   {
      joystickController = getFirstJoystickFoundOnSystem();

      if (joystickController == null)
      {
         throw new IOException("Joystick not found!");
      }
      
      model = JoystickModel.getModelFromName(joystickController.getName());
      
      for (Component component : joystickController.getComponents())
      {
         identifiers.add(component.getIdentifier());
      }
      
      joystickUpdater = new JoystickUpdater(joystickController, statusListeners);

      Thread thread = new Thread(joystickUpdater);
      thread.setPriority(Thread.NORM_PRIORITY);
      thread.start();
   }

   public void addJoystickEventListener(JoystickEventListener joystickEventListener)
   {
      joystickUpdater.addListener(joystickEventListener);
   }
   
   public void addJoystickStatusListener(JoystickStatusListener joystickStatusListener)
   {
      statusListeners.add(joystickStatusListener);
   }
   
   public void clearEventListeners()
   {
      joystickUpdater.clearListeners();
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
   
   public void setPollInterval(int pollIntervalMillis)
   {
      joystickUpdater.setPollIntervalMillis(pollIntervalMillis);
   }
   
   public JoystickModel getModel()
   {
      return model;
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

   public static boolean isAJoystickConnectedToSystem()
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
