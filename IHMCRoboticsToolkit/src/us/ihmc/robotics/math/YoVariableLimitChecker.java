package us.ihmc.robotics.math;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;

public class YoVariableLimitChecker
{
   private final EnumYoVariable<Status> status;
   private final double lowerLimit;
   private final double upperLimit;
   private final DoubleYoVariable variableToCheck;

   public YoVariableLimitChecker(DoubleYoVariable variableToCheck, String prefix, double lowerLimit, double upperLimit, YoVariableRegistry registry)
   {
      status = new EnumYoVariable<Status>(prefix + variableToCheck.getName() + "_Status", registry, Status.class);

      if (upperLimit < lowerLimit)
      {
         System.out.println("YoVariableLimitChecker: Disabling limits. Upper limit needs to be greater than lower limit for variable: "
               + variableToCheck.getName());
         this.lowerLimit = Double.NEGATIVE_INFINITY;
         this.upperLimit = Double.POSITIVE_INFINITY;
      }
      else
      {
         this.lowerLimit = lowerLimit;
         this.upperLimit = upperLimit;
      }

      this.variableToCheck = variableToCheck;
   }

   public Status update()
   {
      if (variableToCheck.getDoubleValue() > upperLimit)
         status.set(Status.ABOVE_LIMIT);
      else if (variableToCheck.getDoubleValue() < lowerLimit)
         status.set(Status.BELOW_LIMIT);
      else
         status.set(Status.IN_RANGE);

      return status.getEnumValue();
   }

   public Status getStatus()
   {
      return status.getEnumValue();
   }

   public enum Status
   {
      IN_RANGE, BELOW_LIMIT, ABOVE_LIMIT
   }
}
