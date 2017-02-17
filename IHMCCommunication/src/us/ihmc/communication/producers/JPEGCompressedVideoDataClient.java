package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.util.ByteBufferProvider;
import us.ihmc.codecs.yuv.JPEGDecoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class JPEGCompressedVideoDataClient implements CompressedVideoDataClient
{
   private final JPEGDecoder decoder = new JPEGDecoder();
   private final VideoStreamer videoStreamer;
   private final ByteBufferProvider byteBufferProvider = new ByteBufferProvider();
   private final YUVPictureConverter converter = new YUVPictureConverter();
   public JPEGCompressedVideoDataClient(VideoStreamer videoStreamer)
   {
      this.videoStreamer = videoStreamer;
   }

   @Override
   public void connected()
   {

   }

   @Override
   public void disconnected()
   {

   }

   @Override
   public void consumeObject(byte[] data, Point3D position, Quaternion orientation, IntrinsicParameters intrinsicParameters)
   {
      ByteBuffer buffer = byteBufferProvider.getOrCreateBuffer(data.length);
      buffer.put(data);
      buffer.flip();
      YUVPicture pic = decoder.decode(buffer);
      BufferedImage img = converter.toBufferedImage(pic);
      pic.delete();
      videoStreamer.updateImage(img, position, orientation, intrinsicParameters);
   }
}
