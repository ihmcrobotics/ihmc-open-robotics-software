package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.jMonkeyEngineToolkit.CameraAdapter;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.io.printing.PrintTools;

public class OffscreenBufferVideoServer
{

   private final RenderedSceneHandler videoDataServer;

   private final CameraAdapter camera;

   private final TimestampProvider timestampProvider;

   public OffscreenBufferVideoServer(Graphics3DAdapter adapter, CameraMountList mountList, CameraConfiguration cameraConfiguration,
         CameraTrackingAndDollyPositionHolder cameraTrackingAndDollyPositionHolder, int width, int height, RenderedSceneHandler videoDataServer, 
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
      PrintTools.info(this, "Starting video stream");
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
         double f = bufferedImage.getWidth() / 2 / Math.tan(fov / 2);
         IntrinsicParameters intrinsicParameters = new IntrinsicParameters(f, f, 0, (bufferedImage.getWidth() - 1) / 2f, (bufferedImage.getHeight() - 1) / 2f, bufferedImage.getWidth(), bufferedImage.getHeight());

         videoDataServer.updateImage(RobotSide.LEFT, bufferedImage, timeStamp, cameraPosition, cameraOrientation, intrinsicParameters);
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
         return videoDataServer.isReadyForNewData();
      }

      public long getTimeStamp()
      {
         return timestampProvider.getTimestamp();
      }
   }
}
