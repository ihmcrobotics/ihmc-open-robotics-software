package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class LookAndStepBehaviorParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey stepLength = keys.addDoubleKey("Step length");
   public static final DoubleStoredPropertyKey idealFootstepLengthOverride = keys.addDoubleKey("Ideal footstep length override");
   public static final DoubleStoredPropertyKey planarRegionsExpiration = keys.addDoubleKey("Planar regions expiration");
   public static final DoubleStoredPropertyKey swingTime = keys.addDoubleKey("Swing time");
   public static final DoubleStoredPropertyKey transferTime = keys.addDoubleKey("Transfer time");

   public LookAndStepBehaviorParameters()
   {
      super(keys, LookAndStepBehaviorParameters.class, "ihmc-open-robotics-software", "ihmc-avatar-interfaces/src/behavior-clean-room/resources");
      load();
   }

   public double getPlanarRegionsExpiration()
   {
      return get(planarRegionsExpiration);
   }

   public static void main(String[] args)
   {
      LookAndStepBehaviorParameters parameters = new LookAndStepBehaviorParameters();
      parameters.save();
   }
}
