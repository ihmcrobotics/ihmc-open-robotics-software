package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.jMonkeyEngineToolkit.CameraAdapter;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.image.ImageCallback;

public class OffscreenBufferVideoServer   
{
   private final ImageCallback imageCallback;

   private final CameraAdapter camera;

   private final TimestampProvider timestampProvider;

   public OffscreenBufferVideoServer(Graphics3DAdapter adapter, CameraMountList mountList, CameraConfiguration cameraConfiguration,
         CameraTrackingAndDollyPositionHolder cameraTrackingAndDollyPositionHolder, int width, int height, ImageCallback imageCallback, 
         TimestampProvider timestampProvider, int framesPerSecond)
   {
      ViewportAdapter viewport = adapter.createNewViewport(null, false, true);
      camera = viewport.getCamera();
      viewport.setupOffscreenView(width, height);

      ClassicCameraController cameraController = new ClassicCameraController(adapter, viewport, cameraTrackingAndDollyPositionHolder);
      cameraController.setConfiguration(cameraConfiguration, mountList);
      viewport.setCameraController(cameraController);

      CameraUpdater cameraUpdater = new CameraUpdater();
      this.imageCallback = imageCallback;
      this.timestampProvider = timestampProvider;
      PrintTools.info(this, "Starting video stream");
      viewport.getCaptureDevice().streamTo(cameraUpdater, framesPerSecond);

   }

   public void close()
   {
      imageCallback.dispose();
   }

   private class CameraUpdater implements CameraStreamer
   {
      @Override
      public void updateImage(BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, double fov)
      {
         
         imageCallback.onNewImage(bufferedImage, timeStamp, cameraPosition, cameraOrientation, fov);
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
         return imageCallback.isAvailable();
      }

      @Override
      public long getTimeStamp()
      {
         return timestampProvider.getTimestamp();
      }
   }
}
