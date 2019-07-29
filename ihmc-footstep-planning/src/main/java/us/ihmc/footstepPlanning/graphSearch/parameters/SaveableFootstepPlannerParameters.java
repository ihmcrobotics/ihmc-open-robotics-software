package us.ihmc.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import us.ihmc.tools.property.StoredPropertySet;
import us.ihmc.tools.property.StoredPropertySetBasics;

public interface SaveableFootstepPlannerParameters extends FootstepPlannerParametersBasics
{
   @Override
   StoredPropertySet getStoredPropertySet();
}