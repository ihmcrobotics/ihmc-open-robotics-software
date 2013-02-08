package us.ihmc.graphics3DAdapter.camera;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.graphics3DAdapter.CameraAdapter;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;

public class OffscreenBufferVideoServer
{

   private final CompressedVideoDataServer compressedVideoDataServer;

   private final CameraAdapter camera;


   public OffscreenBufferVideoServer(Graphics3DAdapter adapter, CameraMountList mountList, CameraConfiguration cameraConfiguration,
         CameraTrackingAndDollyPositionHolder cameraTrackingAndDollyPositionHolder, VideoSettings settings, int port)
   {
      ViewportAdapter viewport = adapter.createNewViewport(null, false, true);
      camera = viewport.getCamera();
      viewport.setupOffscreenView(settings.getWidth(), settings.getHeight());

      ClassicCameraController cameraController = new ClassicCameraController(adapter, viewport, cameraTrackingAndDollyPositionHolder);
      cameraController.setConfiguration(cameraConfiguration, mountList);
      viewport.setCameraController(cameraController);

      CameraUpdater cameraUpdater = new CameraUpdater();
      compressedVideoDataServer = new CompressedVideoDataServer(settings, port);
      

      viewport.getCaptureDevice().streamTo(cameraUpdater, 25);

   }

   public void close()
   {
      compressedVideoDataServer.close();
   }

   private class CameraUpdater implements CameraStreamer
   {

      public void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, double fov)
      {
         compressedVideoDataServer.updateImage(bufferedImage, cameraPosition, cameraOrientation, fov);
      }

      public Point3d getCameraPosition()
      {
         return camera.getCameraPosition();
      }

      public Quat4d getCameraOrientation()
      {
         return camera.getCameraRotation();
      }

      public double getFieldOfView()
      {
         return camera.getHorizontalFovInRadians();
      }

      public boolean isReadyForNewData()
      {
         return compressedVideoDataServer.isConnected();
      }
      
      
   }
}
