package us.ihmc.robotics.math.filters;

public interface ProcessingYoVariable
{
   public abstract void update();

   public default void reset()
   {
   }
}
