package us.ihmc.robotics.math;

import static us.ihmc.robotics.MathTools.square;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

public class YoRMSCalculator
{
   private final DoubleYoVariable rms;
   private final IntegerYoVariable nUpdates;

   public YoRMSCalculator(String prefix, YoVariableRegistry registry)
   {
      rms = new DoubleYoVariable(prefix + "Rms", registry);
      nUpdates = new IntegerYoVariable(prefix + "RmsNUpdates", registry);
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
