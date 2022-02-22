package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.codecs.generated.RGBPicture;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.util.ByteBufferProvider;
import us.ihmc.codecs.yuv.JPEGDecoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;

public class JPEGDecompressor
{
   private final JPEGDecoder jpegDecoder = new JPEGDecoder();
   private final ByteBufferProvider byteBufferProvider = new ByteBufferProvider();
   private final ByteBufferProvider byteBufferProvider2 = new ByteBufferProvider();
   private final YUVPictureConverter yuvPictureConverter = new YUVPictureConverter();

   public BufferedImage decompressJPEGDataToBufferedImage(byte[] jpegData)
   {
      ByteBuffer byteBuffer = byteBufferProvider.getOrCreateBuffer(jpegData.length);
      byteBuffer.put(jpegData);
      byteBuffer.flip();
      YUVPicture yuvPicture = jpegDecoder.decode(byteBuffer);
      BufferedImage bufferedImage = yuvPictureConverter.toBufferedImage(yuvPicture);
      yuvPicture.delete();
      return bufferedImage;
   }

   public Triple<ByteBuffer, Integer, Integer> decompressJPEGDataToBGR8ByteBuffer(byte[] jpegData)
   {
      ByteBuffer byteBuffer = byteBufferProvider.getOrCreateBuffer(jpegData.length);
      byteBuffer.put(jpegData);
      byteBuffer.flip();
      YUVPicture yuvPicture = jpegDecoder.decode(byteBuffer);
      RGBPicture rgbPicture = yuvPicture.toRGB();
      ByteBuffer rgbBuffer = byteBufferProvider2.getOrCreateBuffer(yuvPicture.getWidth() * yuvPicture.getHeight() * 3);
      rgbBuffer.put(10, (byte) 31);
      rgbPicture.get(rgbBuffer);
      int width = yuvPicture.getWidth();
      int height = yuvPicture.getHeight();
      rgbPicture.delete();
      yuvPicture.delete();
      return Triple.of(rgbBuffer, width, height);
   }
}
