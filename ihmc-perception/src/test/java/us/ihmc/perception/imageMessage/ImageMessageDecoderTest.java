package us.ihmc.perception.imageMessage;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImageTest;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.cuda.CUDAJPEGProcessor;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.function.BiConsumer;

import static org.junit.jupiter.api.Assertions.*;

public class ImageMessageDecoderTest
{
   private static Mat zedColorBGR;
   private static Mat zedDepth16U;

   @BeforeAll
   public static void loadImages() throws URISyntaxException, IOException
   {
      Path zedColorBGRPath = Path.of(RawImageTest.class.getResource("zedColorBGR.raw").toURI());
      Path zedDepth16UPath = Path.of(RawImageTest.class.getResource("zedDepth16U.raw").toURI());

      byte[] colorBytes = Files.readAllBytes(zedColorBGRPath);
      byte[] depthBytes = Files.readAllBytes(zedDepth16UPath);

      zedColorBGR = new Mat(720, 1280, opencv_core.CV_8UC3, new BytePointer(colorBytes));
      zedDepth16U = new Mat(720, 1280, opencv_core.CV_16UC1, new BytePointer(depthBytes));
   }

   @Test
   public void testPNG()
   {
      for (PixelFormat pixelFormat : PixelFormat.values())
      {  // Test the color formats
         if (pixelFormat != PixelFormat.GRAY16)
         {
            LogTools.info("Testing Color PNG Compression: {}", pixelFormat.name());
            testDecompression(this::opencvPNGCompression, zedColorBGR, pixelFormat, 0.001);
            System.out.println();
         }
      }

      // Test depth compression
      LogTools.info("Testing Depth PNG Compression");
      testDecompression(this::opencvPNGCompression, zedDepth16U, PixelFormat.GRAY16, 0.001);
      System.out.println();
   }

   @Test
   public void testUncompressed()
   {
      for (PixelFormat pixelFormat : PixelFormat.values())
      {  // Test the color formats
         if (pixelFormat != PixelFormat.GRAY16)
         {
            LogTools.info("Testing Color PNG Compression: {}", pixelFormat.name());
            testDecompression(this::packUncompressed, zedColorBGR, pixelFormat, 0.001);
            System.out.println();
         }
      }

      // Test depth compression
      LogTools.info("Testing Depth PNG Compression");
      testDecompression(this::packUncompressed, zedDepth16U, PixelFormat.GRAY16, 0.001);
      System.out.println();
   }

   private void testDecompression(BiConsumer<Mat, ImageMessage> compressionFunction,
                                  Mat inputImage,
                                  PixelFormat encodedPixelFormat,
                                  double decompressionEpsilon)
   {
      Mat colorConvertedInputImage = convertBGRToPixelFormat(inputImage, encodedPixelFormat);
      Mat colorConvertedRGBAInputImage = convertPixelFormatToRGBA(colorConvertedInputImage, encodedPixelFormat);

      ImageMessage message = new ImageMessage();
      message.setImageWidth(colorConvertedInputImage.cols());
      message.setImageHeight(colorConvertedInputImage.rows());
      encodedPixelFormat.packImageMessage(message);

      compressionFunction.accept(colorConvertedInputImage, message);

      ImageMessageDecoder decoder = new ImageMessageDecoder();

      // TEST CPU DECODING
      Mat cpuDecodedImage = new Mat();
      decoder.decodeMessage(message, cpuDecodedImage);

      double averagePixelDifferenceAfterDecoding = OpenCVTools.averagePixelDifference(cpuDecodedImage, colorConvertedInputImage);
      LogTools.info("CPU Difference: {}", averagePixelDifferenceAfterDecoding);
      assertEquals(0.0, averagePixelDifferenceAfterDecoding, decompressionEpsilon);
      cpuDecodedImage.close();

      // TEST GPU DECODING
      GpuMat gpuDecodedImage = new GpuMat();
      Mat gpuDownloadedImage = new Mat();
      decoder.decodeMessage(message, gpuDecodedImage);
      gpuDecodedImage.download(gpuDownloadedImage);

      averagePixelDifferenceAfterDecoding = OpenCVTools.averagePixelDifference(gpuDownloadedImage, colorConvertedInputImage);
      LogTools.info("GPU Difference: {}", averagePixelDifferenceAfterDecoding);
      assertEquals(0.0, averagePixelDifferenceAfterDecoding, decompressionEpsilon);
      gpuDecodedImage.close();
      gpuDownloadedImage.close();

      // TEST CPU RGBA DECODING
      Mat cpuDecodedRGBAImage = new Mat();
      decoder.decodeMessageToRGBA(message, cpuDecodedRGBAImage);

      averagePixelDifferenceAfterDecoding = OpenCVTools.averagePixelDifference(cpuDecodedRGBAImage, colorConvertedRGBAInputImage);
      LogTools.info("CPU RGBA Difference: {}", averagePixelDifferenceAfterDecoding);
      assertEquals(0.0, averagePixelDifferenceAfterDecoding, decompressionEpsilon);
      assertEquals(PixelFormat.RGBA8, decoder.getDecodedImagePixelFormat());
      cpuDecodedRGBAImage.close();

      // TEST GPU RGBA DECODING
      GpuMat gpuDecodedRGBAImage = new GpuMat();
      Mat gpuDownloadedRGBAImage = new Mat();
      decoder.decodeMessageToRGBA(message, gpuDecodedRGBAImage);
      gpuDecodedRGBAImage.download(gpuDownloadedRGBAImage);

      averagePixelDifferenceAfterDecoding = OpenCVTools.averagePixelDifference(gpuDownloadedRGBAImage, colorConvertedRGBAInputImage);
      LogTools.info("GPU RGBA Difference: {}", averagePixelDifferenceAfterDecoding);
      assertEquals(0.0, averagePixelDifferenceAfterDecoding, decompressionEpsilon);
      assertEquals(PixelFormat.RGBA8, decoder.getDecodedImagePixelFormat());
      gpuDecodedRGBAImage.close();
      gpuDownloadedRGBAImage.close();

      colorConvertedInputImage.close();
      colorConvertedRGBAInputImage.close();
      decoder.destroy();
   }

   private void packUncompressed(Mat image, ImageMessage message)
   {
      PerceptionMessageTools.packImageMessageData(message, image.data().limit(OpenCVTools.dataSize(image)));
      CompressionType.UNCOMPRESSED.packImageMessage(message);
   }

   private void opencvPNGCompression(Mat image, ImageMessage message)
   {
      BytePointer pngData = new BytePointer();
      opencv_imgcodecs.imencode(".png", image, pngData);

      PerceptionMessageTools.packImageMessageData(message, pngData);
      CompressionType.PNG.packImageMessage(message);

      pngData.close();
   }

   private void opencvJpegCompression(Mat image, ImageMessage message)
   {
      BytePointer jpegData = new BytePointer();
      opencv_imgcodecs.imencode(".jpg", image, jpegData);

      PerceptionMessageTools.packImageMessageData(message, jpegData);
      CompressionType.JPEG.packImageMessage(message);

      jpegData.close();
   }

   private void hybridDepthCompression(Mat image, ImageMessage message)
   {
      CUDACompressionTools compressionTools = new CUDACompressionTools();

      BytePointer compressedData = compressionTools.compressDepth(image);
      PerceptionMessageTools.packImageMessageData(message, compressedData);
      CompressionType.ZSTD_JPEG_HYBRID.packImageMessage(message);

      compressionTools.destroy();
      compressedData.close();
   }

   private void nvcompCompression(Mat image, ImageMessage message)
   {
      CUDACompressionTools compressionTools = new CUDACompressionTools();

      BytePointer compressedData = compressionTools.compress(image);
      PerceptionMessageTools.packImageMessageData(message, compressedData);
      CompressionType.NVCOMP.packImageMessage(message);

      compressionTools.destroy();
      compressedData.close();
   }

   private void nvjpegColorCompression(Mat image, ImageMessage message)
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor();

      BytePointer encodedData = new BytePointer(OpenCVTools.dataSize(image));
      jpegProcessor.encodeBGR(image, encodedData);

      PerceptionMessageTools.packImageMessageData(message, encodedData);
      CompressionType.JPEG.packImageMessage(message);

      encodedData.close();
      jpegProcessor.destroy();
   }

   private Mat convertBGRToPixelFormat(Mat bgrImage, PixelFormat pixelFormat)
   {
      int colorConversion = bgrToPixelFormat(pixelFormat);
      if (colorConversion < 0)
         return bgrImage.clone();

      Mat newImage = new Mat();
      opencv_imgproc.cvtColor(bgrImage, newImage, colorConversion);
      return newImage;
   }

   private Mat convertPixelFormatToRGBA(Mat image, PixelFormat pixelFormat)
   {
      Mat newImage = new Mat();
      if (image.type() == opencv_core.CV_16UC1)
      {
         OpenCVTools.clampTo8BitUnsignedChar(image, newImage, 0.0, 255.0);
         OpenCVTools.convertGrayToRGBA(newImage, newImage);
      }
      else
      {
         int colorConversion = pixelFormatToRGBA(pixelFormat);
         if (colorConversion < 0)
            image.copyTo(newImage);
         else
            opencv_imgproc.cvtColor(image, newImage, colorConversion);
      }

      return newImage;
   }

   private static int bgrToPixelFormat(PixelFormat pixelFormat)
   {
      return switch (pixelFormat)
      {
         case YUV_I420 -> opencv_imgproc.COLOR_BGR2YUV_I420;
         case BGRA8 -> opencv_imgproc.COLOR_BGR2BGRA;
         case RGB8 -> opencv_imgproc.COLOR_BGR2RGB;
         case RGBA8 -> opencv_imgproc.COLOR_BGR2RGBA;
         case GRAY8 -> opencv_imgproc.COLOR_BGR2GRAY;
         case BGR8, GRAY16 -> -1;
      };
   }

   private static int pixelFormatToRGBA(PixelFormat pixelFormat)
   {
      return switch (pixelFormat)
      {
         case YUV_I420 -> opencv_imgproc.COLOR_YUV2RGBA_I420;
         case BGR8 -> opencv_imgproc.COLOR_BGR2RGBA;
         case BGRA8 -> opencv_imgproc.COLOR_BGRA2RGBA;
         case RGB8 -> opencv_imgproc.COLOR_RGB2RGBA;
         case GRAY8 -> opencv_imgproc.COLOR_GRAY2RGBA;
         case RGBA8, GRAY16 -> -1;
      };
   }
}
