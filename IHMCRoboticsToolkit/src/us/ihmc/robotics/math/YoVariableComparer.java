package us.ihmc.robotics.math;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class YoVariableComparer
{
   private final YoEnum<CompareStatus> status;
   private final YoDouble yoVariableA;
   private final YoDouble yoVariableB;
   private final YoDouble difference;
   private final YoDouble threshold;
   private final double nonChangingThreshold;
   private ThresholdType compareType;

   public YoVariableComparer(YoDouble variableA, YoDouble variableB, String name, ThresholdType thresholdType, YoDouble threshold,
         YoVariableRegistry registry)
   {
      this.yoVariableA = variableA;
      this.yoVariableB = variableB;
      this.compareType = thresholdType;
      this.threshold = threshold;
      this.nonChangingThreshold = Double.NaN;
      this.status = new YoEnum<CompareStatus>(name + "_status", registry, CompareStatus.class);
      this.difference = new YoDouble(name, registry);
   }
   
   public YoVariableComparer(YoDouble variableA, YoDouble variableB, String name, ThresholdType thresholdType, double threshold,
         YoVariableRegistry registry)
   {
      this.yoVariableA = variableA;
      this.yoVariableB = variableB;
      this.compareType = thresholdType;
      this.threshold = null;
      this.nonChangingThreshold = threshold;
      this.difference = new YoDouble(name, registry);
      this.status = new YoEnum<CompareStatus>(name + "_status", registry, CompareStatus.class);
   }

   public void update()
   {
      double threshold = (this.threshold != null) ? this.threshold.getDoubleValue() : nonChangingThreshold;
      double aValue = yoVariableA.getDoubleValue();
      double bValue = yoVariableB.getDoubleValue();

      switch (compareType)
      {
      case DIFFERENCE_GREATER_THAN:
         difference.set(Math.abs(bValue - aValue));
         status.set((difference.getDoubleValue() > threshold) ? CompareStatus.ABOVE_LIMIT : CompareStatus.IN_RANGE);
         break;
      case DIFFERENCE_LESS_THAN:
         difference.set(Math.abs(bValue - aValue));
         status.set((difference.getDoubleValue() < threshold) ? CompareStatus.BELOW_LIMIT : CompareStatus.IN_RANGE);
         break;
      case SQUARED_DIFFERENCE_GREATER_THAN:
         difference.set(Math.pow(bValue - aValue, 2));
         status.set((difference.getDoubleValue() < threshold) ? CompareStatus.BELOW_LIMIT : CompareStatus.IN_RANGE);
         break;
      case SQUARED_DIFFERENCE_LESS_THAN:
         difference.set(Math.pow(bValue - aValue, 2));
         status.set((difference.getDoubleValue() < threshold) ? CompareStatus.BELOW_LIMIT : CompareStatus.IN_RANGE);
         break;
      default:
         break;
      }
   }
   
   public double getDoubleValue()
   {
      return difference.getDoubleValue();
   }

   public CompareStatus getStatus()
   {
      return status.getEnumValue();
   }

   public enum ThresholdType
   {
      DIFFERENCE_GREATER_THAN, DIFFERENCE_LESS_THAN, SQUARED_DIFFERENCE_LESS_THAN, SQUARED_DIFFERENCE_GREATER_THAN
   };

   public enum CompareStatus
   {
      IN_RANGE, BELOW_LIMIT, ABOVE_LIMIT
   }

   public String getName()
   {
      return difference.getName();
   }
}
