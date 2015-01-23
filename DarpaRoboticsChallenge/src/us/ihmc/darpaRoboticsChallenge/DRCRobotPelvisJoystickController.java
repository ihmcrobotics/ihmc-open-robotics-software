package us.ihmc.darpaRoboticsChallenge;

import net.java.games.input.Component;
import net.java.games.input.Component.Identifier;
import net.java.games.input.Controller;
import net.java.games.input.ControllerEnvironment;
import us.ihmc.simulationconstructionset.joystick.DoubleYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionset.joystick.JoystickUpdater;
import us.ihmc.utilities.jinput.JInputLibraryLoader;
import us.ihmc.yoUtilities.dataStructure.YoVariableHolder;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public class DRCRobotPelvisJoystickController
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

   public DRCRobotPelvisJoystickController(YoVariableHolder holder)
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

      DoubleYoVariable desiredCenterOfMassHeight = (DoubleYoVariable) holder.getVariable("desiredCenterOfMassHeight");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredCenterOfMassHeight, findComponent(Component.Identifier.Axis.SLIDER),
              minHeight, maxHeight, deadZone, true));

      DoubleYoVariable desiredPelvisRoll = (DoubleYoVariable) holder.getVariable("desiredPelvisRoll");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredPelvisRoll, findComponent(Component.Identifier.Axis.X), -maxRoll, maxRoll,
              deadZone, false));

      DoubleYoVariable desiredPelvisPitch = (DoubleYoVariable) holder.getVariable("desiredPelvisPitch");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredPelvisPitch, findComponent(Component.Identifier.Axis.Y), -maxPitch,
              maxPitch, deadZone, true));

      DoubleYoVariable desiredHeadingFinal = (DoubleYoVariable) holder.getVariable("desiredHeadingFinal");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredHeadingFinal, findComponent(Component.Identifier.Axis.RZ), -maxYaw, maxYaw,
              deadZone, true));
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
