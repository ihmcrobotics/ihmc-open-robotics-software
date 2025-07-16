package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public abstract class FastLocomotionParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey swingTime = keys.addDoubleKey("Swing time", 0.6);
   public static final DoubleStoredPropertyKey transferTime = keys.addDoubleKey("Transfer time", 0.15);
   public static final DoubleStoredPropertyKey finalTransferTime = keys.addDoubleKey("Final Transfer time", 0.3);

   public FastLocomotionParameters(Class<?> robotSubclassForLoading)
   {
      super(keys, robotSubclassForLoading);
   }

   public double getSwingTime()
   {
      return get(swingTime);
   }

   public double getTransferTime()
   {
      return get(transferTime);
   }

   public double getFinalTransferTime()
   {
      return get(finalTransferTime);
   }
}