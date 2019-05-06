package us.ihmc.footstepPlanning.graphSearch.parameters;

public class FootstepPlannerParameter<T>
{
   private T value;

   public void setValue(T value)
   {
      this.value = value;
   }

   public T getValue()
   {
      return value;
   }
}
