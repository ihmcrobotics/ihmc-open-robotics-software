package us.ihmc.behaviors;

public interface BehaviorInterface
{
   void setEnabled(boolean enabled);

   default void destroy()
   {

   }
}
