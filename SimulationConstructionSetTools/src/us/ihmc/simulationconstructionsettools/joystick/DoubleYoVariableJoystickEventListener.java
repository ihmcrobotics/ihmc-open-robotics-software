package us.ihmc.simulationconstructionsettools.joystick;

import net.java.games.input.Component;
import net.java.games.input.Event;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;


public class DoubleYoVariableJoystickEventListener implements JoystickEventListener
{
   private final DoubleYoVariable variable;
   private final Component component;
   private final double min;
   private final double max;
   private final double deadZone;
   private final int sign;

   public DoubleYoVariableJoystickEventListener(DoubleYoVariable variable, Component component, double min, double max, double deadZone, boolean signFlip)
   {
      if (variable == null)
         throw new RuntimeException("variable is null");
      if (min > max)
         throw new RuntimeException("min > max");
      if (deadZone < 0.0)
         throw new RuntimeException("deadZone < 0.0");
      this.variable = variable;
      this.component = component;
      this.min = min;
      this.max = max;
      this.deadZone = deadZone;
      this.sign = signFlip ? -1 : 1;
   }

   @Override
   public void processEvent(Event event)
   {
      if (event.getComponent() == component)
      {
         double valueSignChanged = sign * event.getValue();
         double valueDeadZoneCompensated = handleDeadZone(valueSignChanged, deadZone);
         double valueScaled = scaleAxisValue(min, max, valueDeadZoneCompensated);
         variable.set(valueScaled);
      }
   }

   private static double handleDeadZone(double value, double deadZone)
   {
      if (Math.abs(value) < deadZone)
         return 0.0;
      else
         return value;
   }

   private static double scaleAxisValue(double min, double max, double value)
   {
      double percentOfAxis = (value - (-1.0f)) / 2.0f;
      double scaleRange = (max - min) * percentOfAxis;
      double scaledValue = min + scaleRange;
      return scaledValue;
   }
}
