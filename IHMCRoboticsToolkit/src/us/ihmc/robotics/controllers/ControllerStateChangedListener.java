package us.ihmc.robotics.controllers;

public interface ControllerStateChangedListener
{
   public void controllerStateHasChanged(Enum<?> oldState, Enum<?> newState);
}
