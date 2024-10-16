package us.ihmc.yoVariables.filters;

public interface ProcessingYoVariable
{
   public abstract void update();

   public default void reset()
   {
   }
}
