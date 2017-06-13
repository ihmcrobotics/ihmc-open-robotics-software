package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.util.ByteBufferProvider;
import us.ihmc.codecs.yuv.JPEGDecoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.video.VideoCallback;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class JPEGCompressedVideoDataClient implements CompressedVideoDataClient
{
   private final JPEGDecoder decoder = new JPEGDecoder();
   private final VideoCallback videoStreamer;
   private final ByteBufferProvider byteBufferProvider = new ByteBufferProvider();
   private final YUVPictureConverter converter = new YUVPictureConverter();
   
   public JPEGCompressedVideoDataClient(VideoCallback videoStreamer)
   {
      this.videoStreamer = videoStreamer;
   }

   @Override
   public void onFrame(VideoSource videoSource, byte[] data, long timestamp, Point3DReadOnly position, QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters)
   {
      ByteBuffer buffer = byteBufferProvider.getOrCreateBuffer(data.length);
      buffer.put(data);
      buffer.flip();
      YUVPicture pic = decoder.decode(buffer);
      BufferedImage img = converter.toBufferedImage(pic);
      pic.delete();
      videoStreamer.onFrame(videoSource, img, timestamp, position, orientation, intrinsicParameters);
   }

   @Override
   public void connected()
   {
   
   }

   @Override
   public void disconnected()
   {
   
   }
}
