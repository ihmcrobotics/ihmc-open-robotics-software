package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.tools.image.ImageCallback;

public class VideoDataServerImageCallback implements ImageCallback
{
   private final VideoDataServer videoDataServer;
   
   public VideoDataServerImageCallback(VideoDataServer videoDataServer)
   {
      this.videoDataServer = videoDataServer;
   }
   
   
   public boolean isAvailable()
   {
      return videoDataServer.isConnected();
   }
   
   public void onNewImage(BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, double fov)
   {
      double f = bufferedImage.getWidth() / 2 / Math.tan(fov / 2);
      IntrinsicParameters intrinsicParameters = new IntrinsicParameters(f, f, 0, (bufferedImage.getWidth() - 1) / 2f, (bufferedImage.getHeight() - 1) / 2f, bufferedImage.getWidth(), bufferedImage.getHeight());

      videoDataServer.onFrame(VideoSource.MULTISENSE_LEFT_EYE, bufferedImage, timeStamp, cameraPosition, cameraOrientation, intrinsicParameters);

   }
   
   public void dispose()
   {
      // Gross
      if (videoDataServer instanceof CompressedVideoDataServer)
      {
         ((CompressedVideoDataServer) videoDataServer).dispose();
      }
   }
   
}
