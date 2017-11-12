package us.ihmc.robotics.math;

import static us.ihmc.commons.MathTools.square;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoRMSCalculator
{
   private final YoDouble rms;
   private final YoInteger nUpdates;

   public YoRMSCalculator(String prefix, YoVariableRegistry registry)
   {
      rms = new YoDouble(prefix + "Rms", registry);
      nUpdates = new YoInteger(prefix + "RmsNUpdates", registry);
   }

   public void update(double input)
   {
      int nUpdatesOld = nUpdates.getIntegerValue();
      nUpdates.increment();
      int nUpdatesNew = nUpdates.getIntegerValue();
      double ratio = ((double) nUpdatesOld) / ((double) nUpdatesNew);
      rms.set(Math.sqrt(square(rms.getDoubleValue()) * ratio + square(input) / nUpdatesNew));
   }

   public double val()
   {
      return rms.getDoubleValue();
   }

   public void reset()
   {
      nUpdates.set(0);
   }
}
