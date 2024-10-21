package us.ihmc.yoVariables.filters;

public interface ProcessingYoVariable
{
   void update();

   default void reset()
   {
   }
}
