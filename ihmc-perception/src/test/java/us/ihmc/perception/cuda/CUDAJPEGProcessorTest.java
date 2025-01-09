package us.ihmc.perception.cuda;

import org.apache.commons.io.IOUtils;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
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
         PerceptionDebugTools.display("Original vs Processed", new MatVector(bgrImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(bgrImage, decodedImage);
         LogTools.info("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0);
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
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(bgrImage));

         jpegProcessor.decodeToBGR(encodedImage, encodedImage.limit(), decodedImage);
         PerceptionDebugTools.display("Original vs Processed", new MatVector(bgrImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(bgrImage, decodedImage);
         LogTools.info("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0);
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
         PerceptionDebugTools.display("Original vs Processed", new MatVector(grayImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(grayImage, decodedImage);
         LogTools.info("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0);
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
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(bgrImage));

         jpegProcessor.decodeToBGR(encodedImage, encodedImage.limit(), decodedImage);
         PerceptionDebugTools.display("Original vs Processed", new MatVector(bgrImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(bgrImage, decodedImage);
         LogTools.info("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0);
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
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(bgrImage));

         jpegProcessor.decodeToYUV(encodedImage, encodedImage.limit(), decodedImage);
         PerceptionDebugTools.display("Original vs Processed", new MatVector(yuvImage, decodedImage), 1000);

         double averageDifference = OpenCVTools.averagePixelDifference(yuvImage, decodedImage);
         LogTools.info("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0);
      }

      jpegProcessor.destroy();
   }

   @Test
   public void tesEncodeYUV420DecodeYUVI()
   {
      CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(100);

      try (Mat bgrImage = readBGRImage();
           Mat yuvImage = new Mat();
           BytePointer encodedImage = new BytePointer(OpenCVTools.dataSize(bgrImage));
           Mat decodedImage = new Mat())

      {
         opencv_imgproc.cvtColor(bgrImage, yuvImage, opencv_imgproc.COLOR_BGR2YUV_I420);

         jpegProcessor.encodeYUVI420(yuvImage, encodedImage);
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(bgrImage));

         jpegProcessor.decodeToYUV(encodedImage, encodedImage.limit(), decodedImage);
         PerceptionDebugTools.display("Original vs Processed", new MatVector(yuvImage, decodedImage), 10000);

         double averageDifference = OpenCVTools.averagePixelDifference(yuvImage, decodedImage);
         LogTools.info("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 5.0);
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
         assertTrue(encodedImage.limit() < OpenCVTools.dataSize(bgrImage));

         jpegProcessor.decodeToBGR(encodedImage, encodedImage.limit(), decodedImage);
         PerceptionDebugTools.display("Original vs Processed", new MatVector(bgrImage, decodedImage), 10000);

         double averageDifference = OpenCVTools.averagePixelDifference(bgrImage, decodedImage);
         LogTools.info("Average Pixel Difference: {}", averageDifference);
         assertTrue(averageDifference < 10.0);
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
