package us.ihmc.simulationconstructionset.gui;

import us.ihmc.jMonkeyEngineToolkit.camera.TrackingDollyCameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;

public class ViewportAdapterAndCameraControllerHolder
{
   private ViewportAdapter viewportAdapter;
   private TrackingDollyCameraController trackingDollyCameraController;

   public ViewportAdapterAndCameraControllerHolder(ViewportAdapter viewportAdapter, TrackingDollyCameraController trackingDollyCameraController)
   {
      this.viewportAdapter = viewportAdapter;
      this.trackingDollyCameraController = trackingDollyCameraController;
   }

   public ViewportAdapter getViewportAdapter()
   {
      return viewportAdapter;
   }

   public TrackingDollyCameraController getCameraController()
   {
      return trackingDollyCameraController;
   }

   public void closeAndDispose()
   {
      viewportAdapter = null;
      trackingDollyCameraController = null;
   }

}
