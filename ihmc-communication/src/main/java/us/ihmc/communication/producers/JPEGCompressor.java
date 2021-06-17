package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.ByteBuffer;

import us.ihmc.codecs.generated.RGBPicture;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.generated.YUVPicture.YUVSubsamplingType;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;

public class JPEGCompressor
{   
   private final YUVPictureConverter converter = new YUVPictureConverter();
   private final JPEGEncoder encoder = new JPEGEncoder();

   public byte[] convertBufferedImageToJPEGData(BufferedImage bufferedImage)
   {
      YUVPicture picture = converter.fromBufferedImage(bufferedImage, YUVSubsamplingType.YUV420);
      return convertYUVPictureToJPEGData(picture);
   }

   public byte[] convertRGB8ToJPEGData(int width, int height, ByteBuffer rgb8Buffer)
   {
      RGBPicture rgbPicture = new RGBPicture(width, height);
      rgbPicture.putRGBA(rgb8Buffer);
      YUVPicture yuvPicture = rgbPicture.toYUV(YUVSubsamplingType.YUV420);
      rgbPicture.delete();
      return convertYUVPictureToJPEGData(yuvPicture);
   }

   private byte[] convertYUVPictureToJPEGData(YUVPicture picture)
   {
      try
      {
         ByteBuffer buffer = encoder.encode(picture, 75);
         byte[] data = new byte[buffer.remaining()];
         buffer.get(data);
         return data;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }
}
