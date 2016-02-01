package us.ihmc.simulationconstructionset.gui;

import us.ihmc.graphics3DAdapter.camera.CaptureDevice;
import us.ihmc.graphics3DAdapter.camera.TrackingDollyCameraController;

public interface ActiveCanvas3DHolder
{
   public abstract CaptureDevice getActiveCaptureDevice();
   public abstract TrackingDollyCameraController getCamera();
}
