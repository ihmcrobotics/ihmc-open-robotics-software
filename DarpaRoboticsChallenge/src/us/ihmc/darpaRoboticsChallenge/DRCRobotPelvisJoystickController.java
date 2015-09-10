package us.ihmc.darpaRoboticsChallenge;

import net.java.games.input.Component;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.joystick.DoubleYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionset.joystick.JoystickUpdater;


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

   public DRCRobotPelvisJoystickController(YoVariableHolder holder)
   {
      joystickUpdater = new JoystickUpdater();
      joystickUpdater.setPollInterval(pollIntervalMillis);

      Thread thread = new Thread(joystickUpdater);
      thread.setPriority(Thread.NORM_PRIORITY);
      thread.start();

      DoubleYoVariable desiredCenterOfMassHeight = (DoubleYoVariable) holder.getVariable("desiredCenterOfMassHeight");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredCenterOfMassHeight, joystickUpdater.findComponent(Component.Identifier.Axis.SLIDER),
              minHeight, maxHeight, deadZone, true));

      DoubleYoVariable desiredPelvisRoll = (DoubleYoVariable) holder.getVariable("desiredPelvisRoll");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredPelvisRoll, joystickUpdater.findComponent(Component.Identifier.Axis.X), -maxRoll, maxRoll,
              deadZone, false));

      DoubleYoVariable desiredPelvisPitch = (DoubleYoVariable) holder.getVariable("desiredPelvisPitch");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredPelvisPitch, joystickUpdater.findComponent(Component.Identifier.Axis.Y), -maxPitch,
              maxPitch, deadZone, true));

      DoubleYoVariable desiredHeadingFinal = (DoubleYoVariable) holder.getVariable("desiredHeadingFinal");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredHeadingFinal, joystickUpdater.findComponent(Component.Identifier.Axis.RZ), -maxYaw, maxYaw,
              deadZone, true));
   }



  
   
   public static void main(String[] arg)
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
