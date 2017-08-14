package us.ihmc.robotics.controllers;

import us.ihmc.robotics.geometry.FrameVector2D;

public interface ControllerFailureListener
{
   public void controllerFailed(FrameVector2D fallingDirection);
}
