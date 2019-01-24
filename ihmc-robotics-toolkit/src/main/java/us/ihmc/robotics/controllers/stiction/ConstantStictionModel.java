package us.ihmc.robotics.controllers.stiction;

import us.ihmc.yoVariables.providers.DoubleProvider;

public class ConstantStictionModel implements StictionModel
{
   private final DoubleProvider stictionValue;

   public ConstantStictionModel(DoubleProvider stictionValue)
   {
      this.stictionValue = stictionValue;
   }

   @Override
   public double getStictionMagnitude()
   {
      return stictionValue.getValue();
   }
}
