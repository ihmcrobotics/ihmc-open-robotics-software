package us.ihmc.footstepPlanning.graphSearch.parameters;

import java.util.List;

public class FootstepPlannerParameterKey<T>
{
   private final String name;
   private final int id;

   public FootstepPlannerParameterKey(List<FootstepPlannerParameterKey<?>> listToAddTo, int id, String name)
   {
      this.id = id;
      this.name = name;

      listToAddTo.add(this);
   }

   public String getName()
   {
      return name;
   }
}
