package us.ihmc.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;
import us.ihmc.tools.property.YoStoredPropertySet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoFootstepPlannerParameters implements FootstepPlannerParametersBasics
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoStoredPropertySet storedPropertySet;



   public YoFootstepPlannerParameters(YoVariableRegistry parentRegistry, FootstepPlannerParametersReadOnly defaults)
   {
      storedPropertySet = new YoStoredPropertySet(FootstepPlannerParameterKeys.keys, registry);
      parentRegistry.addChild(registry);

      set(defaults);
   }

   @Override
   public StoredPropertySetBasics getStoredPropertySet()
   {
      return storedPropertySet;
   }
}
