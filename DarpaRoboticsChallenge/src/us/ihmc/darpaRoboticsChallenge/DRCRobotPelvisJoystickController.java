package us.ihmc.darpaRoboticsChallenge;

import java.io.IOException;

import net.java.games.input.Component;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.joystick.DoubleYoVariableJoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.Joystick;


public class DRCRobotPelvisJoystickController
{
   private double deadZone = 0.05;

   private double maxPitch = Math.toRadians(20.0);
   private double maxRoll = Math.toRadians(20.0);
   private double maxYaw = Math.toRadians(30.0);
   private double maxHeight = 1.3;
   private double minHeight = 0.5;
   private final int pollIntervalMillis = 20;

   private final Joystick joystick;

   public DRCRobotPelvisJoystickController(YoVariableHolder holder) throws IOException
   {
      joystick = new Joystick();
      joystick.setPollInterval(pollIntervalMillis);

      DoubleYoVariable desiredCenterOfMassHeight = (DoubleYoVariable) holder.getVariable("desiredCenterOfMassHeight");
      joystick.addJoystickEventListener(new DoubleYoVariableJoystickEventListener(desiredCenterOfMassHeight, joystick.findComponent(Component.Identifier.Axis.SLIDER),
              minHeight, maxHeight, deadZone, true));

      DoubleYoVariable desiredPelvisRoll = (DoubleYoVariable) holder.getVariable("desiredPelvisRoll");
      joystick.addJoystickEventListener(new DoubleYoVariableJoystickEventListener(desiredPelvisRoll, joystick.findComponent(Component.Identifier.Axis.X), -maxRoll, maxRoll,
              deadZone, false));

      DoubleYoVariable desiredPelvisPitch = (DoubleYoVariable) holder.getVariable("desiredPelvisPitch");
      joystick.addJoystickEventListener(new DoubleYoVariableJoystickEventListener(desiredPelvisPitch, joystick.findComponent(Component.Identifier.Axis.Y), -maxPitch,
              maxPitch, deadZone, true));

      DoubleYoVariable desiredHeadingFinal = (DoubleYoVariable) holder.getVariable("desiredHeadingFinal");
      joystick.addJoystickEventListener(new DoubleYoVariableJoystickEventListener(desiredHeadingFinal, joystick.findComponent(Component.Identifier.Axis.RZ), -maxYaw, maxYaw,
              deadZone, true));
   }
   
   public static void main(String[] arg) throws IOException
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      VariableChangedListener listener = new VariableChangedListener()
      {
         
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            System.out.println("Change: " + v.getShortName() + " -> " + v.getValueAsDouble());            
         }
      };
      DoubleYoVariable desiredCenterOfMassHeight = new DoubleYoVariable("desiredCenterOfMassHeight", registry);
      desiredCenterOfMassHeight.addVariableChangedListener(listener);
      DoubleYoVariable desiredPelvisRoll = new DoubleYoVariable("desiredPelvisRoll", registry);
      desiredPelvisRoll.addVariableChangedListener(listener);
      DoubleYoVariable desiredPelvisPitch = new DoubleYoVariable("desiredPelvisPitch", registry);
      desiredPelvisPitch.addVariableChangedListener(listener);
      DoubleYoVariable desiredHeadingFinal = new DoubleYoVariable("desiredHeadingFinal", registry);
      desiredHeadingFinal.addVariableChangedListener(listener);
      new DRCRobotPelvisJoystickController(registry);
   }
}
