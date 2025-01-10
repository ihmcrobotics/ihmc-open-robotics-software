package us.ihmc.perception.cuda;

import org.apache.commons.io.IOUtils;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImageTest;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.io.IOException;
import java.net.URL;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

public class CUDAJPEGProcessorTest
{
   private static final boolean SHOW_COMPARISON = false;

   @Test
   public void testEncodeBGRDecodeBGR()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         jpegProcessor.encodeBGR(bgrImage, encodedImage);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(bgrImage));

         jpegProcessor.decodeToBGR(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(bgrImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(bgrImage, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void testEncodeRGBDecodeBGR()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat rgbImage = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         opencv_imgproc.cvtColor(bgrImage, rgbImage, opencv_imgproc.COLOR_BGR2RGB);

         jpegProcessor.encodeRGB(rgbImage, encodedImage);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(rgbImage));

         jpegProcessor.decodeToBGR(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(bgrImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(bgrImage, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void testEncodeBGRDecodeGray()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat grayImage = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         opencv_imgproc.cvtColor(bgrImage, grayImage, opencv_imgproc.COLOR_BGR2GRAY);

         jpegProcessor.encodeBGR(bgrImage, encodedImage);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(bgrImage));

         jpegProcessor.decodeToGray(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(grayImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(grayImage, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void testEncodeYUVDecodeBGR()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat yuvImage = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         opencv_imgproc.cvtColor(bgrImage, yuvImage, opencv_imgproc.COLOR_BGR2YUV);

         jpegProcessor.encodeYUV444(yuvImage, encodedImage);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(yuvImage));

         jpegProcessor.decodeToBGR(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(bgrImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(bgrImage, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void testEncodeYUV444DecodeYUV()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat yuvImage = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         opencv_imgproc.cvtColor(bgrImage, yuvImage, opencv_imgproc.COLOR_BGR2YUV);

         jpegProcessor.encodeYUV444(yuvImage, encodedImage);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(yuvImage));

         jpegProcessor.decodeToYUV(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(yuvImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(yuvImage, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void testEncodeYUVI420DecodeYUV()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat yuvImage = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         opencv_imgproc.cvtColor(bgrImage, yuvImage, opencv_imgproc.COLOR_BGR2YUV_I420);

         jpegProcessor.encodeYUVI420(yuvImage, encodedImage);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(yuvImage));

         jpegProcessor.decodeToYUV(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(yuvImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(yuvImage, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void tesEncodeYUVI420DecodeBGR()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat yuvImage = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         opencv_imgproc.cvtColor(bgrImage, yuvImage, opencv_imgproc.COLOR_BGR2YUV_I420);

         jpegProcessor.encodeYUVI420(yuvImage, encodedImage);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(yuvImage));

         jpegProcessor.decodeToBGR(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(bgrImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(bgrImage, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 10.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void tesEncodeGrayDecodeGray()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat grayImage = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         opencv_imgproc.cvtColor(bgrImage, grayImage, opencv_imgproc.COLOR_BGR2GRAY);

         jpegProcessor.encodeGray(grayImage, encodedImage);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(grayImage));

         jpegProcessor.decodeToGray(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(grayImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(grayImage, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void testEncodeGrayDecodeBGR()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat grayBGRImage = new Mat();
           Mat grayImage = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         opencv_imgproc.cvtColor(bgrImage, grayImage, opencv_imgproc.COLOR_BGR2GRAY);
         opencv_imgproc.cvtColor(grayImage, grayBGRImage, opencv_imgproc.COLOR_GRAY2BGR);

         jpegProcessor.encodeGray(grayImage, encodedImage);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(grayImage));

         jpegProcessor.decodeToBGR(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(grayBGRImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(grayBGRImage, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void testEncodeOpenCVYUVI420DecodeNVJPEGGray()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat yuv420Image = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         opencv_imgproc.cvtColor(bgrImage, yuv420Image, opencv_imgproc.COLOR_BGR2YUV_I420);
         opencv_imgcodecs.imencode(".jpg", yuv420Image, encodedImage, OpenCVTools.compressionParametersJPG);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(yuv420Image));

         jpegProcessor.decodeToGray(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(yuv420Image, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(yuv420Image, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void testEncodeOpenCVBGRDecodeNVJPEGYUV()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat yuvI420Image = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         opencv_imgproc.cvtColor(bgrImage, yuvI420Image, opencv_imgproc.COLOR_BGR2YUV_I420);
         opencv_imgcodecs.imencode(".jpg", bgrImage, encodedImage, OpenCVTools.compressionParametersJPG);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(bgrImage));

         jpegProcessor.decodeToYUV(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(yuvI420Image, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(yuvI420Image, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 10.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void testEncodeOpenCVBGRDecodeNVJPEGBGR()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())
      {
         opencv_imgcodecs.imencode(".jpg", bgrImage, encodedImage, OpenCVTools.compressionParametersJPG);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(bgrImage));

         jpegProcessor.decodeToBGR(encodedImage, encodedImage.limit(), decodedImage);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(bgrImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(bgrImage, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 10.0, "Average Pixel Difference: " + averageDifference);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void testEncodeNVJPEGDecodeOpenCV()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat yuv420Image = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage)))
      {
         opencv_imgproc.cvtColor(bgrImage, yuv420Image, opencv_imgproc.COLOR_BGR2YUV_I420);
         jpegProcessor.encodeYUVI420(yuv420Image, encodedImage);

         Mat jpegMat = new Mat(1, (int) encodedImage.limit(), opencv_core.CV_8UC1, encodedImage);
         Mat decodedImage = opencv_imgcodecs.imdecode(jpegMat, opencv_imgcodecs.IMREAD_UNCHANGED);
         if (SHOW_COMPARISON)
            PerceptionDebugTools.display("Original vs Processed", new MatVector(bgrImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(bgrImage, decodedImage);
         LogTools.debug("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 10.0, "Average Pixel Difference: " + averageDifference);

         decodedImage.close();
      }

      jpegProcessor.destroy();
   }

   private Mat readBGRImage()
   {
      try
      {
         URL imageURL = RawImageTest.class.getResource("zedColorBGR.raw");
         byte[] imageBytes = IOUtils.toByteArray(Objects.requireNonNull(imageURL));
         return new Mat(720, 1280, opencv_core.CV_8UC3, new BytePointer(imageBytes));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
