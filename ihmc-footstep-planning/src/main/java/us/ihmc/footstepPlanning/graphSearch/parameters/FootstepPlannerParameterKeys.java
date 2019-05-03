package us.ihmc.footstepPlanning.graphSearch.parameters;

import java.util.List;

public interface FootstepPlannerParameterKeys
{
   List<FootstepPlannerParameterKey<?>> getKeys();

   String getSaveFileName();
}
