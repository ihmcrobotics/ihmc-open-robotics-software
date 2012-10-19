package us.ihmc.darpaRoboticsChallenge;

import net.java.games.input.Component;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableHolder;
import com.yobotics.simulationconstructionset.joystick.DoubleYoVariableJoystickEventListener;
import com.yobotics.simulationconstructionset.joystick.JoystickUpdater;


public class JinputJoystickController
{
   private double deadZone = 0.05;

   private double maxPitch = Math.toRadians(20.0);
   private double maxRoll = Math.toRadians(20.0);
   private double maxYaw = Math.toRadians(30.0);
   private double maxHeight = 1.3;
   private double minHeight = 0.5;
   private final int pollIntervalMillis = 20;

   private final JoystickUpdater joystickUpdater;
   private final Controller joystickController;

   public JinputJoystickController(YoVariableHolder holder)
   {
      joystickController = findController();
      if (joystickController == null)
      {
         throw new RuntimeException("joystick not found");
      }

//      for (Component component : joystickController.getComponents())
//      {
//         System.out.println(component.getName());
//      }

      joystickUpdater = new JoystickUpdater(joystickController);
      joystickUpdater.setPollInterval(pollIntervalMillis);

      Thread thread = new Thread(joystickUpdater);
      thread.setPriority(Thread.NORM_PRIORITY);
      thread.start();

      DoubleYoVariable desiredCenterOfMassHeight = (DoubleYoVariable) holder.getVariable("desiredCenterOfMassHeight");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredCenterOfMassHeight, findComponent("slider"), minHeight, maxHeight,
              deadZone, true));

      DoubleYoVariable desiredPelvisRoll = (DoubleYoVariable) holder.getVariable("desiredPelvisRoll");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredPelvisRoll, findComponent("x"), -maxRoll, maxRoll,
              deadZone, false));

      DoubleYoVariable desiredPelvisPitch = (DoubleYoVariable) holder.getVariable("desiredPelvisPitch");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredPelvisPitch, findComponent("y"), -maxPitch, maxPitch,
              deadZone, true));

      DoubleYoVariable desiredHeadingFinal = (DoubleYoVariable) holder.getVariable("desiredHeadingFinal");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredHeadingFinal, findComponent("rz"), -maxYaw, maxYaw,
              deadZone, true));
   }

   /**
    * Just return the first stick you find
    */
   private Controller findController()
   {
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

   private Component findComponent(String name)
   {
      for (Component component : joystickController.getComponents())
      {
         if (component.getName().equals(name))
            return component;
      }

      throw new RuntimeException("component " + name + " not found");
   }
}
