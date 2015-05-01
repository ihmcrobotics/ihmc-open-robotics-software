package us.ihmc.simulationconstructionset.util.simulationRunner;

import us.ihmc.utilities.math.geometry.FrameVector2d;

public interface ControllerFailureListener
{
   public void controllerFailed(FrameVector2d fallingDirection);
}
