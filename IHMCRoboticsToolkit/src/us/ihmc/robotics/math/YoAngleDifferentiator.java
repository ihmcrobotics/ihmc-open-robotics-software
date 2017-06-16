package us.ihmc.robotics.math;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.AngleTools;

public class YoAngleDifferentiator
{
   private boolean hasBeenUpdated = false;
   private final YoDouble previous;
   private double output = Double.NaN;
   private double dt;

   public YoAngleDifferentiator(String name, double dt, YoVariableRegistry registry)
   {
      this.previous = new YoDouble(name, registry);
      this.dt = dt;
   }

   public void update(double input)
   {
      if (hasBeenUpdated)
      {
         output = (AngleTools.computeAngleDifferenceMinusPiToPi(input, previous.getDoubleValue())) / dt;
         previous.set(input);
      }
      else
      {
         reset(input);
      }

      hasBeenUpdated = true;
   }

   public double val()
   {
      return output;
   }

   public void reset(double resetVal)
   {
      previous.set(resetVal);
      output = 0.0;
   }
}
