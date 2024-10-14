package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

class VariableTools
{
   static YoBoolean createHasBeenCalledYoBoolean(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      return new YoBoolean(namePrefix + "HasBeenCalled" + nameSuffix, registry);
   }

   static YoBoolean createLimitedCalledYoBoolean(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      return new YoBoolean(namePrefix + "Limited" + nameSuffix, registry);
   }

   static DoubleProvider createMaxRateYoDouble(String namePrefix, String nameSuffix, double initialValue, YoRegistry registry)
   {
      YoDouble maxRate = new YoDouble(namePrefix + "MaxRate" + nameSuffix, registry);
      maxRate.set(initialValue);
      return maxRate;
   }

   static DoubleProvider createAlphaYoDouble(String namePrefix, String nameSuffix, double initialValue, YoRegistry registry)
   {
      YoDouble maxRate = new YoDouble(namePrefix + "AlphaVariable" + nameSuffix, registry);
      maxRate.set(initialValue);
      return maxRate;
   }

   static YoInteger createWindowSizeYoInteger(String namePrefix, String nameSuffix, int initialValue, YoRegistry registry)
   {
      YoInteger windowSize = new YoInteger(namePrefix + "WindowSize" + nameSuffix, registry);
      windowSize.set(initialValue);
      return windowSize;
   }

   static DoubleProvider createMaxAccelerationYoDouble(String namePrefix, String nameSuffix, double initialValue, YoRegistry registry)
   {
      YoDouble maxRate = new YoDouble(namePrefix + "MaxAcceleration" + nameSuffix, registry);
      maxRate.set(initialValue);
      return maxRate;
   }
}
