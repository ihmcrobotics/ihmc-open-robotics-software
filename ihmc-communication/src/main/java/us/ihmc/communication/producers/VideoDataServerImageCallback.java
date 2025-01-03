package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.WritableRaster;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.image.ImageCallback;

@Deprecated
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
      bufferedImage = copy(bufferedImage); // FIXME: SCS reuse the same instance
      double f = bufferedImage.getWidth() / 2 / Math.tan(fov / 2);

      videoDataServer.onFrame(VideoSource.MULTISENSE_LEFT_EYE, bufferedImage, timeStamp, cameraPosition, cameraOrientation, new Object());

   }

   private static BufferedImage copy(BufferedImage in)
   {
      ColorModel cm = in.getColorModel();
      boolean isAlphaPremultiplied = cm.isAlphaPremultiplied();
      WritableRaster raster = in.copyData(null);
      return new BufferedImage(cm, raster, isAlphaPremultiplied, null);
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
