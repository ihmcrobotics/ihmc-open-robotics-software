package us.ihmc.simulationconstructionset.util.simulationRunner;

import us.ihmc.robotics.geometry.FrameVector2d;

public interface ControllerFailureListener
{
   public void controllerFailed(FrameVector2d fallingDirection);
}
