package us.ihmc.behaviors.javafx.model;

@FunctionalInterface
public interface FXUIAction
{
   void doAction(FXUITrigger trigger);
}
