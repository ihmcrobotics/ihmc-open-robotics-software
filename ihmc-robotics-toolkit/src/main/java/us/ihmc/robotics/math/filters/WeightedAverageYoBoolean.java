package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

public class WeightedAverageYoBoolean extends YoBoolean
{
   private final List<DoubleProvider> booleanWeights = new ArrayList<>();
   private final List<BooleanProvider> booleansToAverage = new ArrayList<>();

   private final boolean biasToPositive;

   public WeightedAverageYoBoolean(String name, YoVariableRegistry registry)
   {
      this(name, registry, false);
   }

   public WeightedAverageYoBoolean(String name, YoVariableRegistry registry, boolean biasToPositive)
   {
      super(name, "WeightedAverageYoBoolean", registry);

      this.biasToPositive = biasToPositive;
   }

   public void addBooleanToAverage(DoubleProvider booleanWeight, BooleanProvider booleanToAverage)
   {
      booleanWeights.add(booleanWeight);
      booleansToAverage.add(booleanToAverage);
   }

   public boolean update()
   {
      int numberOfBooleans = booleanWeights.size();

      double totalWeight = 0.0;
      double estimatedValue = 0.0;
      for (int i = 0; i < numberOfBooleans; i++)
      {
         double weight = booleanWeights.get(i).getValue();
         estimatedValue += weight * (booleansToAverage.get(i).getValue() ? 1.0 : 0.0);
         totalWeight += weight;
      }

      if (totalWeight <= 0.0)
         throw new RuntimeException("Invalid weights in the weighted average variable.");

      estimatedValue /= totalWeight;

      return evaluate(estimatedValue);
   }

   private boolean evaluate(double estimatedValue)
   {
      if (biasToPositive)
      {
         if (estimatedValue >= 0.5)
         {
            set(true);
            return true;
         }
         else
         {
            set(false);
            return false;
         }
      }
      else
      {
         if (estimatedValue > 0.5)
         {
            set(true);
            return true;
         }
         else
         {
            set(false);
            return false;
         }
      }
   }
}
