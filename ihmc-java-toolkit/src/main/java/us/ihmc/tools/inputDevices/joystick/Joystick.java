package us.ihmc.tools.inputDevices.joystick;

import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.concurrent.ThreadFactory;

import javax.swing.JFrame;

import org.apache.commons.lang3.SystemUtils;

import com.google.common.base.Supplier;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;

/**
 * <p>
 * WARNING: On Windows, the process running this thread needs to have a Window in focus for the OS
 * to reveal events. Call setStandalone() to open a focus window if you're on Windows and don't have
 * any other GUIs running.
 * </p>
 */
public class Joystick
{
   private static final ThreadFactory namedThreadFactory = ThreadTools.createNamedThreadFactory(Joystick.class.getSimpleName());

   private static ControllerEnvironment controllerEnvironment = null;
   private static final Supplier<ControllerEnvironment> controllerEnvironmentFactory = createControllerEnvironmentFactory();

   private static Supplier<ControllerEnvironment> createControllerEnvironmentFactory()
   {
      Constructor<?> constructor;
      try
      {
         constructor = Class.forName("net.java.games.input.DefaultControllerEnvironment").getDeclaredConstructors()[0];
      }
      catch (SecurityException | ClassNotFoundException e1)
      {
         e1.printStackTrace();
         throw new RuntimeException(e1);
      }

      constructor.setAccessible(true);

      return () ->
      {
         try
         {
            return (ControllerEnvironment) constructor.newInstance();
         }
         catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
         {
            e.printStackTrace();
            throw new RuntimeException(e);
         }
      };

   }

   private final ArrayList<JoystickStatusListener> statusListeners = new ArrayList<JoystickStatusListener>();
   private final HashSet<Identifier> identifiers = new HashSet<Identifier>();
   protected final Controller joystickController;
   private final JoystickUpdater joystickUpdater;
   private final JoystickModel model;

   /**
    * Connects to the first joystick found on the system.
    * 
    * @throws IOException
    */
   public Joystick() throws JoystickNotFoundException
   {
      try
      {
         joystickController = getFirstJoystickFoundOnSystem();
      }
      catch (JoystickNotFoundException joystickNotFoundException)
      {
         throw joystickNotFoundException;
      }

      if (getControllers().length > 1)
      {
         LogTools.warn("More than one joystick found!");
         printListOfConnectedJoysticks();
      }

      LogTools.info("Using joystick: " + joystickController.getName());

      model = JoystickModel.getModelFromName(joystickController.getName());

      initializeIdentifiers();

      joystickUpdater = new JoystickUpdater(joystickController, model, statusListeners);

      startThread();
   }

   public static void rescanControllers()
   {
      destroyControllerEnvironment();
   }

   private static void destroyControllerEnvironment()
   {
      Thread.getAllStackTraces().keySet().stream().filter(thread -> thread.getClass().getPackage().getName().equals("net.java.games.input"))
            .forEach(jinputThread ->
            {
               jinputThread.interrupt();
            });

      if (controllerEnvironment != null)
         controllerEnvironment = null;
   }

   private static Controller[] getControllers()
   {
      if (controllerEnvironment == null)
         controllerEnvironment = controllerEnvironmentFactory.get();
      return controllerEnvironment.getControllers();
   }

   /**
    * Connects to the nth joystick of model type found on the system.
    * 
    * @param joystickModel
    * @param indexFoundOnSystem
    */
   public Joystick(JoystickModel joystickModel, int indexFoundOnSystem) throws JoystickNotFoundException
   {
      try
      {
         joystickController = getJoystickOfModelOnSystem(joystickModel, indexFoundOnSystem);
      }
      catch (JoystickNotFoundException joystickNotFoundException)
      {
         throw joystickNotFoundException;
      }

      LogTools.info("Using joystick: " + joystickController.getName());

      model = JoystickModel.getModelFromName(joystickController.getName());

      initializeIdentifiers();

      joystickUpdater = new JoystickUpdater(joystickController, model, statusListeners);

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

      LogTools.info("Using joystick: " + joystickController.getName());

      model = JoystickModel.getModelFromName(joystickController.getName());

      initializeIdentifiers();

      joystickUpdater = new JoystickUpdater(joystickController, model, statusListeners);

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
      Thread updaterThread = namedThreadFactory.newThread(joystickUpdater);
      updaterThread.start();
   }

   public void shutdown()
   {
      LogTools.warn("Joystick (" + joystickController.getName() + ") poll thread shutting down");
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

   /**
    * For Windows users, jinput requires a window to be in focus.
    */
   public void setStandalone()
   {
      if (SystemUtils.IS_OS_WINDOWS)
      {
         JFrame frame = new JFrame("Joystick Input Enable Window");
         frame.setSize(480, 320);
         frame.setVisible(true);
         frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      }
   }

   public void setCustomizationFilter(JoystickCustomizationFilter componentFilter)
   {
      joystickUpdater.setCustomizationFilter(componentFilter);
   }

   public JoystickModel getModel()
   {
      return model;
   }

   private static Controller getFirstJoystickFoundOnSystem() throws JoystickNotFoundException
   {
      Controller[] controllers = getControllers();

      for (Controller controller : controllers)
      {
         if (isValidControllerType(controller))
         {
            return controller;
         }
      }

      throw new JoystickNotFoundException("No joysticks found on system!");
   }

   public void printListOfConnectedJoysticks()
   {
      Controller[] controllers = getControllers();

      for (Controller controller : controllers)
      {
         if (isValidControllerType(controller))
         {
            LogTools.info("Found: " + controller.getName());
         }
      }
   }

   public static boolean isJoystickComboFoundOnSystem(JoystickModel model1, JoystickModel model2)
   {
      Controller[] controllers = getControllers();

      int occurancesOfModel1 = 0;
      int occurancesOfModel2 = 0;
      for (Controller controller : controllers)
      {
         if (isValidControllerType(controller))
         {
            if (JoystickModel.getModelFromName(controller.getName()) == model1)
            {
               ++occurancesOfModel1;
            }
            if (JoystickModel.getModelFromName(controller.getName()) == model2)
            {
               ++occurancesOfModel2;
            }
         }
      }

      if (model1 == model2)
      {
         return occurancesOfModel1 >= 2;
      }
      else
      {
         return occurancesOfModel1 >= 1 && occurancesOfModel2 >= 1;
      }
   }

   private static Controller getJoystickOfModelOnSystem(JoystickModel model, int nthToPick) throws JoystickNotFoundException
   {
      Controller[] controllers = getControllers();

      int i = 0;
      for (Controller controller : controllers)
      {
         if (isValidControllerType(controller))
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

   private static boolean isValidControllerType(Controller controller)
   {
      return controller.getType() == Controller.Type.STICK || controller.getType() == Controller.Type.GAMEPAD;
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
