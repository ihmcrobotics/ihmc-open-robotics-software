package us.ihmc.graphics3DAdapter.camera;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.concurrent.ConcurrentLinkedDeque;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.graphics3DAdapter.CameraAdapter;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.utilities.io.streamingData.StreamingDataConsumer;
import us.ihmc.utilities.io.streamingData.StreamingDataProducer;

public class OffscreenBufferVideoServer
{

   private final CompressedVideoDataServer videoDataCompressor;

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

      try
      {
         videoDataCompressor = new CompressedVideoDataServer(settings, port);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e.getMessage());
      }

      CameraUpdater cameraUpdater = new CameraUpdater();
      

      viewport.getCaptureDevice().streamTo(cameraUpdater, 25);

   }

   public void close()
   {
      videoDataCompressor.close();
   }

   private class CameraUpdater implements StreamingDataProducer, CameraStreamer
   {
      private ConcurrentLinkedDeque<StreamingDataConsumer> consumers = new ConcurrentLinkedDeque<StreamingDataConsumer>();

      public void registerConsumer(StreamingDataConsumer consumer)
      {
         consumers.add(consumer);
      }

      public long getDataIdentifier()
      {
         return 7797525979L;
      }

      public void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, double fov)
      {
         videoDataCompressor.updateImage(bufferedImage, cameraPosition, cameraOrientation, fov);
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
   }
}
