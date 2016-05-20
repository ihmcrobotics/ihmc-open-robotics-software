package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.util.ByteBufferProvider;
import us.ihmc.codecs.yuv.JPEGDecoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;

public class JPEGDecompressor
{
   private final JPEGDecoder jpegDecoder = new JPEGDecoder();
   private final ByteBufferProvider byteBufferProvider = new ByteBufferProvider();
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
}
