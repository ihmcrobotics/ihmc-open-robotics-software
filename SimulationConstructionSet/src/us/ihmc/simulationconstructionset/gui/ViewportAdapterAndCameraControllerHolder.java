package us.ihmc.simulationconstructionset.gui;

import us.ihmc.graphics3DAdapter.camera.TrackingDollyCameraController;
import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;

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
