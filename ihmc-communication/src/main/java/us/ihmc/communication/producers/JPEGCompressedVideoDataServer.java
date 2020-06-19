package us.ihmc.communication.producers;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.ByteBuffer;

import boofcv.struct.calib.CameraPinholeBrown;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.generated.YUVPicture.YUVSubsamplingType;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class JPEGCompressedVideoDataServer implements CompressedVideoDataServer
{
   private static final Object hackyLockBecauseJPEGEncoderIsNotThreadsafe = new Object();

   private final YUVPictureConverter converter = new YUVPictureConverter();
   private final JPEGEncoder encoder = new JPEGEncoder();
   private final CompressedVideoHandler handler;

   private boolean videoEnabled = true;
   private int qualityFactor = 75;
   private boolean cropVideo = false;
   private int cropX;
   private int cropY;
   private int desiredFPS = 25;
   private int fps = 0;

   private long previousTimestamp = -1;

   public JPEGCompressedVideoDataServer(CompressedVideoHandler handler)
   {
      this.handler = handler;
   }

   public void setQualityFactor(int qualityFactor)
   {
      this.qualityFactor = qualityFactor;
   }

   @Override
   public void onFrame(VideoSource videoSource, BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition,
                       QuaternionReadOnly cameraOrientation, CameraPinholeBrown intrinsicParameters)
   {
      if (!handler.isConnected() || !videoEnabled || bufferedImage == null)
         return;

      if (desiredFPS != fps)
      {
         fps = desiredFPS;
      }

      if (previousTimestamp > -1 && (timeStamp - previousTimestamp) < Conversions.secondsToNanoseconds(1.0 / ((double) fps)))
      {
         return;
      }

      if (cropVideo)
      {
         int width = bufferedImage.getWidth();
         int height = bufferedImage.getHeight();

         int newWidth = width / 2;
         int newHeight = height / 2;

         int x = (newWidth * cropX) / 100;
         int y = (newHeight * cropY) / 100;

         BufferedImage croppedImage = new BufferedImage(newWidth, newHeight, bufferedImage.getType());
         Graphics2D graphics = croppedImage.createGraphics();
         graphics.drawImage(bufferedImage, 0, 0, newWidth, newHeight, x, y, x + newWidth, y + newHeight, null);
         graphics.dispose();
         bufferedImage = croppedImage;
      }

      YUVPicture picture = converter.fromBufferedImage(bufferedImage, YUVSubsamplingType.YUV420);

      try
      {
         ByteBuffer buffer;
         synchronized (hackyLockBecauseJPEGEncoderIsNotThreadsafe)
         {
            buffer = encoder.encode(picture, qualityFactor);
         }
         byte[] data = new byte[buffer.remaining()];
         buffer.get(data);
         handler.onFrame(videoSource, data, timeStamp, cameraPosition, cameraOrientation, intrinsicParameters);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      previousTimestamp = timeStamp;
      picture.delete();
   }

   @Override
   public boolean isConnected()
   {
      return handler.isConnected();
   }

   @Override
   public void setVideoControlSettings(VideoControlSettings settings)
   {
      if (settings.isSendVideo())
      {
         qualityFactor = settings.getQualityFactor();
         desiredFPS = settings.getFPS();
         cropVideo = settings.isCropped();
         videoEnabled = true;

         cropX = MathTools.clamp(settings.getCropX(), 0, 100);
         cropY = MathTools.clamp(settings.getCropY(), 0, 100);
      }
      else
      {
         videoEnabled = false;
      }
   }

   @Override
   public void dispose()
   {
   }
}
