package us.ihmc.simulationConstructionSetTools.joystick;

import net.java.games.input.Component;
import net.java.games.input.Event;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.commons.MathTools;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;

public class DoubleYoVariableDPADJoystickEventListener implements JoystickEventListener
{
   private final YoDouble variable;
   private final Component component;
   private final double min;
   private final double max;
   private final double increment;
   private final int sign;

   public DoubleYoVariableDPADJoystickEventListener(YoDouble variable, Component component, double min, double max, double increment, boolean signFlip)
   {
      if (variable == null)
         throw new RuntimeException("variable is null");
      if (min > max)
         throw new RuntimeException("min > max");
      this.variable = variable;
      this.component = component;
      this.min = min;
      this.max = max;
      this.increment = increment;
      this.sign = signFlip ? -1 : 1;
   }

   @Override
   public void processEvent(Event event)
   {
      if (event.getComponent() == component)
      {
         double value = getDPadValue(event);
         double newValue = (sign * value * increment) + variable.getDoubleValue();
         
         newValue = MathTools.clamp(newValue, min, max);
         variable.set(newValue);
      }
   }

   private double getDPadValue(Event event)
   {
      double dPadValue = event.getValue();
      double value = 0.0;
      if (MathTools.epsilonEquals(dPadValue, 0.25, 1e-3))
      {
         value = 1.0;
      }
      if (MathTools.epsilonEquals(dPadValue, 0.75, 1e-3))
      {
         value = -1.0;
      }
      return value;
   }

}
