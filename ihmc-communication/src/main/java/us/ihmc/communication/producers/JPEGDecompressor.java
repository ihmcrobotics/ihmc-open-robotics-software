package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;

import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.codecs.generated.RGBPicture;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.util.ByteBufferProvider;
import us.ihmc.codecs.yuv.JPEGDecoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;

/**
 * Decodes 3 component YUV color MJPEG
 * Will handle YUV444, YUV422, or YUV420
 * See https://bitbucket.ihmc.us/projects/LIBS/repos/ihmc-video-codecs/browse/csrc/JPEGDecoderImpl.cpp
 *
 * Adapted from https://chromium.googlesource.com/libyuv/libyuv/+/refs/heads/main/docs/formats.md:
 * The following formats contains a full size Y plane followed by 2 planes for UV.
 * The size (subsampling) of the UV varies.
 *   I420 is half width, half height
 *   I422 is half width, full height
 *   I444 is full width, full height
 * The YUV formats start with a letter to specify the color space. e.g. I420
 *   I = BT.601 limited range
 *
 * YUVPicture toRGB is doing a YUV -> ARGB and then a ARGB -> RGB24
 * RGB24 is B,G,R in memory
 * See https://bitbucket.ihmc.us/projects/LIBS/repos/ihmc-video-codecs/browse/csrc/YUVPicture.cpp#166
 * See https://chromium.googlesource.com/libyuv/libyuv/+/refs/heads/main/docs/formats.md#rgb24-and-raw
 *
 */
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
