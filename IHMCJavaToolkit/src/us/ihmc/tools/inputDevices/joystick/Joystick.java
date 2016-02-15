package us.ihmc.tools.inputDevices.joystick;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.tools.io.printing.PrintTools;

public class Joystick
{
   private final ArrayList<JoystickStatusListener> statusListeners = new ArrayList<JoystickStatusListener>();
   private final HashSet<Identifier> identifiers = new HashSet<Identifier>();
   protected final Controller joystickController;
   private final JoystickUpdater joystickUpdater;
   private final JoystickModel model;
   private Thread updaterThread;

   /**
    * Connects to the first joystick found on the system.
    * 
    * @throws IOException
    */
   public Joystick() throws IOException
   {
      try
      {
         joystickController = getFirstJoystickFoundOnSystem();
      }
      catch (JoystickNotFoundException joystickNotFoundException)
      {
         throw joystickNotFoundException;
      }
      
      if (ControllerEnvironment.getDefaultEnvironment().getControllers().length > 1)
      {
         PrintTools.warn("More than one joystick found!");
         printListOfConnectedJoysticks();
      }
   
      PrintTools.info("Using joystick: " + joystickController.getName());
      
      model = JoystickModel.getModelFromName(joystickController.getName());
      
      initializeIdentifiers();
      
      joystickUpdater = new JoystickUpdater(joystickController, statusListeners);

      startThread();
   }
   
   /**
    * Connects to the nth joystick of model type found on the system.
    * 
    * @param joystickModel
    * @param nthJoystick
    */
   public Joystick(JoystickModel joystickModel, int nthJoystick) throws IOException
   {
      try
      {
         joystickController = getNthJoystickOfModelOnSystem(joystickModel, nthJoystick);
      }
      catch (JoystickNotFoundException joystickNotFoundException)
      {
         throw joystickNotFoundException;
      }
      
      PrintTools.info("Using joystick: " + joystickController.getName());
      
      model = JoystickModel.getModelFromName(joystickController.getName());
      
      initializeIdentifiers();
      
      joystickUpdater = new JoystickUpdater(joystickController, statusListeners);

      startThread();
   }
   
   /**
    * Connects to the provided controller. For use with VirtualJoystickController.
    * 
    * @param controller to use
    */
   protected Joystick(Controller joystickController) throws IOException
   {      
      this.joystickController = joystickController;
      
      PrintTools.info("Using joystick: " + joystickController.getName());
      
      model = JoystickModel.getModelFromName(joystickController.getName());
      
      initializeIdentifiers();
      
      joystickUpdater = new JoystickUpdater(joystickController, statusListeners);

      startThread();
   }

   private void initializeIdentifiers()
   {
      for (Component component : joystickController.getComponents())
      {
         identifiers.add(component.getIdentifier());
      }
   }

   private void startThread()
   {
      updaterThread = new Thread(joystickUpdater);
      updaterThread.setPriority(Thread.NORM_PRIORITY);
      updaterThread.start();
   }
   
   public void shutdown()
   {
      PrintTools.warn("Joystick (" + joystickController.getName() + ") poll thread shutting down");
      joystickUpdater.stopThread();
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

   private static Controller getFirstJoystickFoundOnSystem() throws JoystickNotFoundException
   {
      Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();
   
      for (Controller controller : controllers)
      {
         if (controller.getType() == Controller.Type.STICK)
         {
            return controller;
         }
      }
      
      throw new JoystickNotFoundException("No joysticks found on system!");
   }
   
   public void printListOfConnectedJoysticks()
   {
      Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();
      
      for (Controller controller : controllers)
      {
         if (controller.getType() == Controller.Type.STICK)
         {
            PrintTools.info(this, "Found: " + controller.getName());
         }
      }
   }
   
   private static Controller getNthJoystickOfModelOnSystem(JoystickModel model, int nthToPick) throws JoystickNotFoundException
   {
      Controller[] controllers = ControllerEnvironment.getDefaultEnvironment().getControllers();
   
      int i = 0;
      for (Controller controller : controllers)
      {
         if (controller.getType() == Controller.Type.STICK)
         {
            if (JoystickModel.getModelFromName(controller.getName()) == model)
            {
               if (i == nthToPick)
               {
                  return controller;
               }
               ++i;
            }
         }
      }
      
      throw new JoystickNotFoundException("Can't find index " + nthToPick + " of model: " + model);
   }

   public static boolean isAJoystickConnectedToSystem()
   {
      try
      {
         getFirstJoystickFoundOnSystem();
         return true;
      }
      catch (JoystickNotFoundException e)
      {
         return false;
      }
   }

   public static void main(String[] args)
   {
      try
      {
         new Joystick();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
