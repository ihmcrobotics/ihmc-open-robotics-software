package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.ByteBuffer;

import us.ihmc.codecs.generated.RGBPicture;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.generated.YUVPicture.YUVSubsamplingType;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;

/**
 * Encode an image to YUV I420 and encode it into a MJPEG.
 * See JPEGDecompressor for more info.
 */
public class JPEGCompressor
{   
   private final YUVPictureConverter converter = new YUVPictureConverter();
   private final JPEGEncoder encoder = new JPEGEncoder();

   public byte[] convertBufferedImageToJPEGData(BufferedImage bufferedImage)
   {
      YUVPicture picture = converter.fromBufferedImage(bufferedImage, YUVSubsamplingType.YUV420);
      return convertYUVPictureToJPEGData(picture);
   }

   /**
    * putRGBA calls libyuv::ARGBToRGB24(src, width * 4, buffer, width * 3, width, height);
    * See https://bitbucket.ihmc.us/projects/LIBS/repos/ihmc-video-codecs/browse/csrc/RGBPicture.cpp#14
    * RGB24 is B,G,R in memory
    * See https://chromium.googlesource.com/libyuv/libyuv/+/refs/heads/main/docs/formats.md#rgb24-and-raw
    * See https://chromium.googlesource.com/libyuv/libyuv/+/refs/heads/main/source/convert_from_argb.cc#1300
    */
   public byte[] convertBGRA8ToJPEGData(int width, int height, ByteBuffer bgra8Buffer)
   {
      RGBPicture rgbPicture = new RGBPicture(width, height);
      rgbPicture.putRGBA(bgra8Buffer);
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
