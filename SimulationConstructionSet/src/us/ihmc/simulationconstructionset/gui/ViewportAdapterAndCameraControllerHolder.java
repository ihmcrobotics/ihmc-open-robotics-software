package us.ihmc.simulationconstructionset.gui;

import us.ihmc.graphics3DAdapter.camera.TrackingDollyCameraController;
import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;

public class ViewportAdapterAndCameraControllerHolder
{
   private final ViewportAdapter viewportAdapter;
   private final TrackingDollyCameraController trackingDollyCameraController;

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

}
