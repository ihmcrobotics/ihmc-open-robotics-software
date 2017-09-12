package us.ihmc.simulationconstructionset.gui;

import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;
import us.ihmc.jMonkeyEngineToolkit.camera.TrackingDollyCameraController;

public interface ActiveCanvas3DHolder
{
   public abstract CaptureDevice getActiveCaptureDevice();
   public abstract TrackingDollyCameraController getCamera();
}
