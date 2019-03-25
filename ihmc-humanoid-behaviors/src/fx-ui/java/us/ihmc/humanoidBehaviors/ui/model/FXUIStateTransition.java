package us.ihmc.humanoidBehaviors.ui.model;

@FunctionalInterface
public interface FXUIStateTransition
{
   void transition(FXUIStateTransitionTrigger trigger);
}
