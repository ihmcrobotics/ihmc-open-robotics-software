package us.ihmc.humanoidBehaviors;

public interface BehaviorInterface
{
   void setEnabled(boolean enabled);

   default void destroy()
   {

   }
}
