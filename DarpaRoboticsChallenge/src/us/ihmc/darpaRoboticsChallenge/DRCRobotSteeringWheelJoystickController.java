package us.ihmc.darpaRoboticsChallenge;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.joystick.DoubleYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionset.joystick.JoystickUpdater;
import us.ihmc.tools.inputDevices.JInputLibraryLoader;


public class DRCRobotSteeringWheelJoystickController
{
   private double deadZone = 0.05;

   private double maxSteeringAngle = Math.toRadians(45.0);
   
   private final int pollIntervalMillis = 20;

   private final JoystickUpdater joystickUpdater;
   private final Controller joystickController;

   public DRCRobotSteeringWheelJoystickController(YoVariableHolder holder)
   {
      joystickController = findController();

      if (joystickController == null)
      {
         throw new RuntimeException("joystick not found");
      }

      joystickUpdater = new JoystickUpdater(joystickController);
      joystickUpdater.setPollInterval(pollIntervalMillis);

      Thread thread = new Thread(joystickUpdater);
      thread.setPriority(Thread.NORM_PRIORITY);
      thread.start();


      DoubleYoVariable desiredSteeringWheelAngle = (DoubleYoVariable) holder.getVariable("desiredSteeringWheelAngle");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredSteeringWheelAngle, findComponent(Component.Identifier.Axis.X), -maxSteeringAngle, maxSteeringAngle,
              deadZone, false));

   }

   /**
    * Just return the first stick you find
    */
   private Controller findController()
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

      return null;
   }

   private Component findComponent(Identifier identifier)
   {
      for (Component component : joystickController.getComponents())
      {
         if (component.getIdentifier().equals(identifier))
            return component;
      }

      throw new RuntimeException("component with identifier " + identifier + " not found");
   }
}
