package us.ihmc.tools.inputDevices.joystick;

import java.util.ArrayList;
import java.util.HashSet;

import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import net.java.games.input.Component.Identifier;
import us.ihmc.tools.io.printing.PrintTools;

public class Joystick
{
   private final ArrayList<JoystickEventListener> eventListeners = new ArrayList<JoystickEventListener>();
   private final ArrayList<JoystickGeneralListener> generalListeners = new ArrayList<JoystickGeneralListener>();
   private final HashSet<Identifier> identifiers = new HashSet<Identifier>();
   private final Controller joystickController;
   private JoystickUpdater joystickUpdater;
   private JoystickModel model;

   public Joystick()
   {
      joystickController = getFirstJoystickFoundOnSystem();

      if (nullCheck())
      {
         PrintTools.error("Joystick not found!");
         return;
      }
      
      model = JoystickModel.getModelFromName(joystickController.getName());
      
      for (Component component : joystickController.getComponents())
      {
         identifiers.add(component.getIdentifier());
      }
      
      joystickUpdater = new JoystickUpdater(joystickController, eventListeners, generalListeners);

      Thread thread = new Thread(joystickUpdater);
      thread.setPriority(Thread.NORM_PRIORITY);
      thread.start();
   }

   private boolean nullCheck()
   {
      if (joystickController == null)
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   public void addJoystickEventListener(JoystickEventListener joystickEventListener)
   {
      if (nullCheck())
         return;
      
      eventListeners.add(joystickEventListener);
   }
   
   public void addJoystickGeneralListener(JoystickGeneralListener joystickGeneralListener)
   {
      if (nullCheck())
         return;
      
      generalListeners.add(joystickGeneralListener);
   }
   
   public void clearEventListeners()
   {
      if (nullCheck())
         return;
      
      eventListeners.clear();
   }

   public Component findComponent(Identifier identifier) throws JoystickComponentNotFoundException
   {
      if (nullCheck())
         return null;
      
      if (hasComponent(identifier))
         return joystickController.getComponent(identifier);
      else
         throw new JoystickComponentNotFoundException(identifier);
   }

   public boolean hasComponent(Identifier identifier)
   {
      if (nullCheck())
         return false;
      
      return identifiers.contains(identifier);
   }
   
   public void setPollInterval(int pollIntervalMillis)
   {
      if (nullCheck())
         return;
      
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
