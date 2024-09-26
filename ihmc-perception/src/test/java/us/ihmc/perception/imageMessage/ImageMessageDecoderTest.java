package us.ihmc.perception.imageMessage;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
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
   private static final double LOSSY_COMPRESSION_EPSILON = 3.0;
   private static final double LOSSLESS_COMPRESSION_EPSILON = 1E-6;

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
            testDecompression(this::opencvPNGCompression, zedColorBGR, pixelFormat, LOSSLESS_COMPRESSION_EPSILON);
            System.out.println();
         }
      }

      // Test depth compression
      LogTools.info("Testing Depth PNG Compression");
      testGeneralDecompression(this::opencvPNGCompression, zedDepth16U, PixelFormat.GRAY16, LOSSLESS_COMPRESSION_EPSILON);
      System.out.println();
   }

   @Test
   public void testUncompressed()
   {
      for (PixelFormat pixelFormat : PixelFormat.values())
      {  // Test the color formats
         if (pixelFormat != PixelFormat.GRAY16)
         {
            LogTools.info("Testing Color Uncompressed: {}", pixelFormat.name());
            testDecompression(this::packUncompressed, zedColorBGR, pixelFormat, LOSSLESS_COMPRESSION_EPSILON);
            System.out.println();
         }
      }

      // Test depth compression
      LogTools.info("Testing Depth Uncompressed");
      testGeneralDecompression(this::packUncompressed, zedDepth16U, PixelFormat.GRAY16, LOSSLESS_COMPRESSION_EPSILON);
      System.out.println();
   }

   @Test
   public void testOpenCVJpeg()
   {
      for (PixelFormat pixelFormat : PixelFormat.values())
      {  // Only test the color formats for jpeg
         if (pixelFormat != PixelFormat.GRAY16)
         {
            LogTools.info("Testing Color JPEG Compression: {}", pixelFormat.name());
            testDecompression(this::opencvJpegCompression, zedColorBGR, pixelFormat, LOSSY_COMPRESSION_EPSILON);
            System.out.println();
         }
      }
   }

   @Test
   public void testNVJPEG()
   {  // Only these formats have been tested to work with NVJPEG. A few other formats (like YUV_I420) might also work, but have not been verified.
      PixelFormat[] nvjpegSupportedFormats = {PixelFormat.BGR8, PixelFormat.RGB8, PixelFormat.GRAY8};

      for (PixelFormat pixelFormat : nvjpegSupportedFormats)
      {
         LogTools.info("Testing NVJPEG Compression: {}", pixelFormat);
         testDecompression(getNVJPEGCompressionFunction(pixelFormat), zedColorBGR, pixelFormat, LOSSY_COMPRESSION_EPSILON);
      }
      System.out.println();
   }

   @Test
   public void testNVCOMP()
   {
      for (PixelFormat pixelFormat : PixelFormat.values())
      {  // Test the color formats
         if (pixelFormat != PixelFormat.GRAY16)
         {
            LogTools.info("Testing Color NVCOMP Compression: {}", pixelFormat.name());
            testDecompression(this::nvcompCompression, zedColorBGR, pixelFormat, LOSSLESS_COMPRESSION_EPSILON);
            System.out.println();
         }
      }

      // Test depth compression
      LogTools.info("Testing Depth NVCOMP Compression");
      testGeneralDecompression(this::nvcompCompression, zedDepth16U, PixelFormat.GRAY16, LOSSLESS_COMPRESSION_EPSILON);
      System.out.println();
   }

   @Test
   public void testHybridDepthCompression()
   {
      LogTools.info("Testing Hybri Depth Compression");
      testGeneralDecompression(this::hybridDepthCompression, zedDepth16U, PixelFormat.GRAY16, LOSSY_COMPRESSION_EPSILON);
      System.out.println();
   }

   private void testDecompression(BiConsumer<Mat, ImageMessage> compressionFunction,
                                  Mat inputImage,
                                  PixelFormat encodedPixelFormat,
                                  double decompressionEpsilon)
   {  // Test both general decompression and decompression to RGBA
      testGeneralDecompression(compressionFunction, inputImage, encodedPixelFormat, decompressionEpsilon);
      testDecompressionToRGBA(compressionFunction, inputImage, encodedPixelFormat, decompressionEpsilon);
   }

   // This test works for both color and depth images
   private void testGeneralDecompression(BiConsumer<Mat, ImageMessage> compressionFunction,
                                         Mat image,
                                         PixelFormat encodingInputPixelFormat,
                                         double decompressionEpsilon)
   {  // Ensure encoder input is in desired pixel format
      Mat encoderInputImage = new Mat();
      if (image.type() == opencv_core.CV_16UC1)
         image.copyTo(encoderInputImage);
      else
         PixelFormat.BGR8.convertToPixelFormat(image, encoderInputImage, encodingInputPixelFormat);
      Mat expectedImage = new Mat();

      // Create and pack parts of the image message
      ImageMessage message = new ImageMessage();
      message.setImageWidth(encoderInputImage.cols());
      message.setImageHeight(encoderInputImage.rows());
      encodingInputPixelFormat.packImageMessage(message);

      // Compress the image and pack into image message
      compressionFunction.accept(encoderInputImage, message);

      // Create a decoder
      ImageMessageDecoder decoder = new ImageMessageDecoder();

      // TEST CPU DECODING
      Mat cpuDecodedImage = new Mat();
      decoder.decodeMessage(message, cpuDecodedImage);
      encodingInputPixelFormat.convertToPixelFormat(encoderInputImage, expectedImage, decoder.getDecodedImagePixelFormat());

      double averagePixelDifferenceAfterDecoding = OpenCVTools.averagePixelDifference(cpuDecodedImage, expectedImage);
      LogTools.info("CPU Difference: {}", averagePixelDifferenceAfterDecoding);
      assertEquals(0.0, averagePixelDifferenceAfterDecoding, decompressionEpsilon);
      cpuDecodedImage.close();

      // TEST GPU DECODING
      GpuMat gpuDecodedImage = new GpuMat();
      Mat gpuDownloadedImage = new Mat();
      decoder.decodeMessage(message, gpuDecodedImage);
      gpuDecodedImage.download(gpuDownloadedImage);
      encodingInputPixelFormat.convertToPixelFormat(encoderInputImage, expectedImage, decoder.getDecodedImagePixelFormat());

      averagePixelDifferenceAfterDecoding = OpenCVTools.averagePixelDifference(gpuDownloadedImage, expectedImage);
      LogTools.info("GPU Difference: {}", averagePixelDifferenceAfterDecoding);
      assertEquals(0.0, averagePixelDifferenceAfterDecoding, decompressionEpsilon);
      gpuDecodedImage.close();
      gpuDownloadedImage.close();

      // close stuff
      encoderInputImage.close();
      expectedImage.close();
      decoder.destroy();
   }

   // This test only works with color images (depth cannot be converted to RGBA without being hacky)
   private void testDecompressionToRGBA(BiConsumer<Mat, ImageMessage> compressionFunction,
                                        Mat image,
                                        PixelFormat encodedPixelFormat,
                                        double decompressionEpsilon)
   {  // Ensure encoder input is in desired pixel format
      Mat encoderInputImage = new Mat();
      PixelFormat.BGR8.convertToPixelFormat(image, encoderInputImage, encodedPixelFormat);
      Mat expectedImage = new Mat();

      ImageMessage message = new ImageMessage();
      message.setImageWidth(encoderInputImage.cols());
      message.setImageHeight(encoderInputImage.rows());
      encodedPixelFormat.packImageMessage(message);

      // Compress the image and pack into the message
      compressionFunction.accept(encoderInputImage, message);

      ImageMessageDecoder decoder = new ImageMessageDecoder();

      // TEST CPU RGBA DECODING
      Mat cpuDecodedRGBAImage = new Mat();
      decoder.decodeMessageToRGBA(message, cpuDecodedRGBAImage);
      encodedPixelFormat.convertToPixelFormat(encoderInputImage, expectedImage, decoder.getDecodedImagePixelFormat());

      double averagePixelDifferenceAfterDecoding = OpenCVTools.averagePixelDifference(cpuDecodedRGBAImage, expectedImage);
      LogTools.info("CPU RGBA Difference: {}", averagePixelDifferenceAfterDecoding);
      assertEquals(0.0, averagePixelDifferenceAfterDecoding, decompressionEpsilon);
      assertEquals(PixelFormat.RGBA8, decoder.getDecodedImagePixelFormat());
      cpuDecodedRGBAImage.close();

      // TEST GPU RGBA DECODING
      GpuMat gpuDecodedRGBAImage = new GpuMat();
      Mat gpuDownloadedRGBAImage = new Mat();
      decoder.decodeMessageToRGBA(message, gpuDecodedRGBAImage);
      gpuDecodedRGBAImage.download(gpuDownloadedRGBAImage);
      encodedPixelFormat.convertToPixelFormat(encoderInputImage, expectedImage, decoder.getDecodedImagePixelFormat());

      averagePixelDifferenceAfterDecoding = OpenCVTools.averagePixelDifference(gpuDownloadedRGBAImage, expectedImage);
      LogTools.info("GPU RGBA Difference: {}", averagePixelDifferenceAfterDecoding);
      assertEquals(0.0, averagePixelDifferenceAfterDecoding, decompressionEpsilon);
      assertEquals(PixelFormat.RGBA8, decoder.getDecodedImagePixelFormat());
      gpuDecodedRGBAImage.close();
      gpuDownloadedRGBAImage.close();

      // Close stuff
      encoderInputImage.close();
      expectedImage.close();
      decoder.destroy();
   }

   private void packUncompressed(Mat image, ImageMessage message)
   {  // Limit of image data must be set correctly to pack uncompressed data
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

   private BiConsumer<Mat, ImageMessage> getNVJPEGCompressionFunction(PixelFormat pixelFormat)
   {
      return (image, message) ->
      {
         CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor();

         BytePointer encodedData = new BytePointer(OpenCVTools.dataSize(image));
         switch (pixelFormat)
         {
            case GRAY8 -> jpegProcessor.encodeGray(image, encodedData);
            case BGR8 -> jpegProcessor.encodeBGR(image, encodedData);
            case RGB8 -> jpegProcessor.encodeRGB(image, encodedData);
            default -> throw new UnsupportedOperationException();
         }

         PerceptionMessageTools.packImageMessageData(message, encodedData);
         CompressionType.NVJPEG.packImageMessage(message);

         encodedData.close();
         jpegProcessor.destroy();
      };
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

   private void hybridDepthCompression(Mat image, ImageMessage message)
   {
      CUDACompressionTools compressionTools = new CUDACompressionTools();

      BytePointer compressedData = compressionTools.compressDepth(image);
      PerceptionMessageTools.packImageMessageData(message, compressedData);
      CompressionType.ZSTD_NVJPEG_HYBRID.packImageMessage(message);

      compressionTools.destroy();
      compressedData.close();
   }
}
