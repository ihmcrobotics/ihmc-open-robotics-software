package us.ihmc.robotics.controllers;

import us.ihmc.euclid.referenceFrame.FrameVector2D;

public interface ControllerFailureListener
{
   public void controllerFailed(FrameVector2D fallingDirection);
}
