package us.ihmc.humanoidBehaviors.javafx.model;

@FunctionalInterface
public interface FXUIAction
{
   void doAction(FXUITrigger trigger);
}
