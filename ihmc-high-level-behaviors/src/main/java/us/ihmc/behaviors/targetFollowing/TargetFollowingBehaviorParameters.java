package us.ihmc.behaviors.targetFollowing;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class TargetFollowingBehaviorParameters extends StoredPropertySet implements TargetFollowingBehaviorParametersReadOnly
{
   public static final String PROJECT_NAME = "ihmc-open-robotics-software";
   public static final String TO_RESOURCE_FOLDER = "ihmc-high-level-behaviors/src/main/resources";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public TargetFollowingBehaviorParameters()
   {
      super(keys, TargetFollowingBehaviorParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      load();
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys, TargetFollowingBehaviorParameters.class, PROJECT_NAME, TO_RESOURCE_FOLDER);
      parameters.loadUnsafe();
      parameters.save();
   }
}
