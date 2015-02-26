package us.ihmc.graphics3DAdapter.camera;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.graphics3DAdapter.CameraAdapter;
import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.utilities.TimestampProvider;
import us.ihmc.utilities.VideoDataServer;
import us.ihmc.utilities.robotSide.RobotSide;

public class OffscreenBufferVideoServer
{

   private final VideoDataServer videoDataServer;

   private final CameraAdapter camera;

   private final TimestampProvider timestampProvider;

   public OffscreenBufferVideoServer(Graphics3DAdapter adapter, CameraMountList mountList, CameraConfiguration cameraConfiguration,
         CameraTrackingAndDollyPositionHolder cameraTrackingAndDollyPositionHolder, int width, int height, VideoDataServer videoDataServer, 
         TimestampProvider timestampProvider, int framesPerSecond)
   {
      ViewportAdapter viewport = adapter.createNewViewport(null, false, true);
      camera = viewport.getCamera();
      viewport.setupOffscreenView(width, height);

      ClassicCameraController cameraController = new ClassicCameraController(adapter, viewport, cameraTrackingAndDollyPositionHolder);
      cameraController.setConfiguration(cameraConfiguration, mountList);
      viewport.setCameraController(cameraController);

      CameraUpdater cameraUpdater = new CameraUpdater();
      this.videoDataServer = videoDataServer;
      this.timestampProvider = timestampProvider;
      System.out.println("STarting vidoe stream");
      viewport.getCaptureDevice().streamTo(cameraUpdater, framesPerSecond);

   }

   public void close()
   {
      videoDataServer.close();
   }

   private class CameraUpdater implements CameraStreamer
   {

      public void updateImage(BufferedImage bufferedImage, long timeStamp, Point3d cameraPosition, Quat4d cameraOrientation, double fov)
      {
         videoDataServer.updateImage(RobotSide.LEFT, bufferedImage, timeStamp, cameraPosition, cameraOrientation, fov);
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
         return videoDataServer.isConnected();
      }

      public long getTimeStamp()
      {
         return timestampProvider.getTimestamp();
      }
   }
}
