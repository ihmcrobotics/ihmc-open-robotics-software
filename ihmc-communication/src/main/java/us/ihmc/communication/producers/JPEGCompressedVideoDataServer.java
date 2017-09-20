package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.ByteBuffer;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.generated.YUVPicture.YUVSubsamplingType;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class JPEGCompressedVideoDataServer implements CompressedVideoDataServer
{
   private static final Object hackyLockBecauseJPEGEncoderIsNotThreadsafe = new Object();
   
   private final YUVPictureConverter converter = new YUVPictureConverter();
   private final JPEGEncoder encoder = new JPEGEncoder();
   private final CompressedVideoHandler handler;
   
   public JPEGCompressedVideoDataServer(CompressedVideoHandler handler)
   {
      this.handler = handler;
   }

   @Override
   public void onFrame(VideoSource videoSource, BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, IntrinsicParameters intrinsicParameters)
   {
      YUVPicture picture = converter.fromBufferedImage(bufferedImage, YUVSubsamplingType.YUV420);
      try
      {
         ByteBuffer buffer;
         synchronized (hackyLockBecauseJPEGEncoderIsNotThreadsafe)
         {
            buffer = encoder.encode(picture, 75);
         }
         byte[] data =  new byte[buffer.remaining()];
         buffer.get(data);
         handler.onFrame(videoSource, data, timeStamp, cameraPosition, cameraOrientation, intrinsicParameters);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      picture.delete();
   }

   @Override
   public boolean isConnected()
   {
      return handler.isConnected();
   }

   @Override
   public void setVideoControlSettings(VideoControlSettings object)
   {
      // TODO
   }

   @Override
   public void dispose()
   {
   }
}
