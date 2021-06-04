package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.nio.BufferOverflowException;
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
   private final ByteBufferProvider byteBufferProvider3 = new ByteBufferProvider();
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

   int i = 0;

   public Triple<ByteBuffer, Integer, Integer> decompressJPEGDataToRGBAByteBuffer(byte[] jpegData)
   {
      ByteBuffer byteBuffer = byteBufferProvider.getOrCreateBuffer(jpegData.length);
      byteBuffer.put(jpegData);
      byteBuffer.flip();
      YUVPicture yuvPicture = jpegDecoder.decode(byteBuffer);
      RGBPicture rgbPicture = yuvPicture.toRGB();
      ByteBuffer rgbBuffer = byteBufferProvider2.getOrCreateBuffer(yuvPicture.getWidth() * yuvPicture.getHeight() * 4);
//      rgbaBuffer.put(10, (byte) 31);
      rgbPicture.get(rgbBuffer);
      int width = yuvPicture.getWidth();
      int height = yuvPicture.getHeight();
      ByteBuffer rgbaBuffer = byteBufferProvider3.getOrCreateBuffer(yuvPicture.getWidth() * yuvPicture.getHeight() * 4);

      rgbBuffer.rewind();
      try
      {
         for (int i = 0; i < yuvPicture.getWidth() * yuvPicture.getHeight(); i++)
         {
            rgbaBuffer.put(rgbBuffer.get());
            rgbaBuffer.put(rgbBuffer.get());
            rgbaBuffer.put(rgbBuffer.get());
            rgbaBuffer.put((byte) 255);
         }
      }
      catch (BufferOverflowException e)
      {
         i++;
      }

      rgbPicture.delete();
      yuvPicture.delete();
      return Triple.of(rgbaBuffer, width, height);
   }
}
