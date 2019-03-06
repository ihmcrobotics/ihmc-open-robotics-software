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

   public WeightedAverageYoBoolean(String name, YoVariableRegistry registry)
   {
      super(name, "WeightedAverageYoBoolean", registry);
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
         totalWeight += totalWeight;
      }

      estimatedValue /= totalWeight;

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
}
