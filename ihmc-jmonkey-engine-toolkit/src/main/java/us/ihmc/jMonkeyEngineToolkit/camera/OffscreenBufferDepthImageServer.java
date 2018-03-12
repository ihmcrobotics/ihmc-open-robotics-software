package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.jMonkeyEngineToolkit.CameraAdapter;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.image.DepthImageCallback;

public class OffscreenBufferDepthImageServer   
{
   private final DepthImageCallback depthImageCallback;

   private final CameraAdapter camera;

   private final TimestampProvider timestampProvider;
   
   private final double nearClip;
   private final double farClip;

   public OffscreenBufferDepthImageServer(Graphics3DAdapter adapter, CameraMountList mountList, CameraConfiguration cameraConfiguration,
         CameraTrackingAndDollyPositionHolder cameraTrackingAndDollyPositionHolder, int width, int height, double nearClip, double farClip, DepthImageCallback imageCallback, 
         TimestampProvider timestampProvider, int framesPerSecond)
   {
      ViewportAdapter viewport = adapter.createNewViewport(null, false, true);
      camera = viewport.getCamera();
      viewport.setupOffscreenView(width, height);
      
      this.nearClip = nearClip;
      this.farClip = farClip;

      ClassicCameraController cameraController = new ClassicCameraController(adapter, viewport, cameraTrackingAndDollyPositionHolder);
      cameraController.setConfiguration(cameraConfiguration, mountList);
      viewport.setCameraController(cameraController);

      CameraUpdater cameraUpdater = new CameraUpdater();
      this.depthImageCallback = imageCallback;
      this.timestampProvider = timestampProvider;
      PrintTools.info(this, "Starting RGBD stream");
      viewport.getCaptureDevice().streamTo(cameraUpdater, framesPerSecond);

   }

   public void close()
   {
      depthImageCallback.dispose();
   }

   private class CameraUpdater implements RGBDStreamer
   {
      
      @Override
      public void updateRBGD(BufferedImage image, Point3D32[] coordinates, int[] colors, long timeStamp, Point3DReadOnly cameraPosition,
                             QuaternionReadOnly cameraOrientation, double fov)
      {
         depthImageCallback.onNewImage(image, coordinates, colors, timeStamp, cameraPosition, cameraOrientation, fov);
      }
      
      @Override
      public double getNearClip()
      {
         return nearClip;
      }

      @Override
      public double getFarClip()
      {
         return farClip;
      }

      @Override
      public Point3DReadOnly getCameraPosition()
      {
         return camera.getCameraPosition();
      }

      @Override
      public QuaternionReadOnly getCameraOrientation()
      {
         return camera.getCameraRotation();
      }

      @Override
      public double getFieldOfView()
      {
         return camera.getHorizontalFovInRadians();
      }

      @Override
      public boolean isReadyForNewData()
      {
         return depthImageCallback.isAvailable();
      }

      @Override
      public long getTimeStamp()
      {
         return timestampProvider.getTimestamp();
      }




   }
}
