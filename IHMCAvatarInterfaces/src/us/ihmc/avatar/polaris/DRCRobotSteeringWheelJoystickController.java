package us.ihmc.avatar.polaris;

import java.io.IOException;

import net.java.games.input.Component;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationConstructionSetTools.joystick.DoubleYoVariableJoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.Joystick;


public class DRCRobotSteeringWheelJoystickController
{
   private double deadZone = 0.05;

   private double maxSteeringAngle = Math.toRadians(45.0);
   
   private final int pollIntervalMillis = 20;

   private final Joystick joystick;

   public DRCRobotSteeringWheelJoystickController(YoVariableHolder holder) throws IOException
   {
      joystick = new Joystick();
      joystick.setPollInterval(pollIntervalMillis);

      DoubleYoVariable desiredSteeringWheelAngle = (DoubleYoVariable) holder.getVariable("desiredSteeringWheelAngle");
      joystick.addJoystickEventListener(new DoubleYoVariableJoystickEventListener(desiredSteeringWheelAngle, joystick.findComponent(Component.Identifier.Axis.X), -maxSteeringAngle, maxSteeringAngle,
              deadZone, false));
   }
}
