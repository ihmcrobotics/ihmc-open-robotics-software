package us.ihmc.robotics.math;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoVariableLimitChecker
{
   private final YoEnum<LimitStatus> status;
   private final DoubleProvider lowerLimit;
   private final DoubleProvider upperLimit;
   private final YoDouble variableToCheck;

   public YoVariableLimitChecker(YoDouble variableToCheck, String prefix, DoubleProvider lowerLimit, DoubleProvider upperLimit, YoRegistry registry)
   {
      status = new YoEnum<>(prefix + variableToCheck.getName() + "_Status", registry, LimitStatus.class);

      this.lowerLimit = lowerLimit;
      this.upperLimit = upperLimit;

      this.variableToCheck = variableToCheck;
   }

   public LimitStatus update()
   {
      if (upperLimit.getValue() < lowerLimit.getValue())
      {
         LogTools.warn("Not checking joint limits, since they aren't valid. Upper limit must be greater than lower limit.");
         return null;
      }

      if (variableToCheck.getDoubleValue() > upperLimit.getValue())
         status.set(LimitStatus.ABOVE_LIMIT);
      else if (variableToCheck.getDoubleValue() < lowerLimit.getValue())
         status.set(LimitStatus.BELOW_LIMIT);
      else
         status.set(LimitStatus.IN_RANGE);

      return status.getEnumValue();
   }

   public LimitStatus getStatus()
   {
      return status.getEnumValue();
   }
   
   
   public double getDoubleValue()
   {
      return variableToCheck.getDoubleValue();
   }
   public String getName()
   {
      return variableToCheck.getName();
   }

   public enum LimitStatus
   {
      IN_RANGE, BELOW_LIMIT, ABOVE_LIMIT
   }
}
