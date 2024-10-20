package us.ihmc.perception.cuda;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImageTest;
import us.ihmc.perception.opencv.OpenCVTools;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

public class CUDACompressionToolsTest
{
   private static Path zedColorBGRPath;
   private static Path zedDepth16UPath;

   @BeforeAll
   public static void loadFiles() throws URISyntaxException
   {
      zedColorBGRPath = Path.of(RawImageTest.class.getResource("zedColorBGR.raw").toURI());
      zedDepth16UPath = Path.of(RawImageTest.class.getResource("zedDepth16U.raw").toURI());
   }

   @Test
   public void testCPUDepthCompressionDecompression() throws IOException
   {
      byte[] depthBytes = Files.readAllBytes(zedDepth16UPath);
      long originalDataSize = depthBytes.length;
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16UC1, new BytePointer(depthBytes));

      CUDACompressionTools compressor = new CUDACompressionTools();
      BytePointer compressedDepth = compressor.compressDepth(depthImage);

      // Sum of data size should be less than original data
      assertTrue(compressedDepth.limit() < originalDataSize);

      Mat decompressedDepth = new Mat(depthImage.size(), depthImage.type());
      compressor.decompressDepth(compressedDepth, decompressedDepth);

      double averageDifference = OpenCVTools.averagePixelDifference(depthImage, decompressedDepth);
      LogTools.info("Difference Ratio: {}", averageDifference);
      assertTrue(averageDifference < 5.0); // On average, decoded pixels differ less than 5mm from the original

      compressor.destroy();
      depthImage.close();
      compressedDepth.close();
      decompressedDepth.close();
   }

   @Test
   public void testGPUDepthCompressionDecompression() throws IOException
   {
      byte[] depthBytes = Files.readAllBytes(zedDepth16UPath);
      long originalDataSize = depthBytes.length;
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16UC1, new BytePointer(depthBytes));
      GpuMat gpuDepthImage = new GpuMat(depthImage.size(), depthImage.type());
      gpuDepthImage.upload(depthImage);

      CUDACompressionTools compressor = new CUDACompressionTools();
      BytePointer compressedDepth = compressor.compressDepth(gpuDepthImage);

      // Sum of data size should be less than original data
      assertTrue(compressedDepth.limit() < originalDataSize);

      GpuMat decompressedDepth = new GpuMat(depthImage.size(), depthImage.type());
      compressor.decompressDepth(compressedDepth, decompressedDepth);

      Mat cpuDecompressedDepth = new Mat(720, 1280, opencv_core.CV_16UC1, new Scalar(0.0));
      decompressedDepth.download(cpuDecompressedDepth);
      double averageDifference = OpenCVTools.averagePixelDifference(depthImage, cpuDecompressedDepth);
      LogTools.info("Difference Ratio: {}", averageDifference);
      assertTrue(averageDifference < 5.0); // On average, decoded pixels differ less than 5mm from the original

      compressor.destroy();
      depthImage.close();
      gpuDepthImage.close();
      compressedDepth.close();
      decompressedDepth.close();
      cpuDecompressedDepth.close();
   }

   @Test
   public void testBasicCompression() throws IOException
   {
      // Read ZED color image data
      byte[] colorImageData = Files.readAllBytes(zedColorBGRPath);
      long originalDataSize = colorImageData.length;
      BytePointer originalData = new BytePointer(colorImageData);

      CUDACompressionTools compressor = new CUDACompressionTools();

      // Compress the data
      BytePointer compressedData = compressor.compress(originalData, originalDataSize);
      long compressedDataSize = compressedData.limit();

      // Compressed data should be smaller than original
      assertTrue(compressedDataSize <= originalDataSize);

      // Decompress the data
      BytePointer decompressedData = compressor.decompress(compressedData, compressedDataSize);
      long decompressedDataSize = decompressedData.limit();

      // Decompressed data should be same size as original
      assertEquals(originalDataSize, decompressedDataSize);
      // Lossless compression; decompressed data should be the same as original
      assertTrue(dataEquals(originalData, decompressedData, decompressedDataSize));

      // Free everything
      compressedData.close();
      decompressedData.close();
      compressor.destroy();
      originalData.close();
   }

   @Test
   public void testCPUMatCompression() throws IOException
   {
      // Read ZED depth image data
      byte[] depthImageData = Files.readAllBytes(zedDepth16UPath);
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16U, new BytePointer(depthImageData));
      long originalDataSize = depthImage.elemSize() * depthImage.total();

      CUDACompressionTools compressor = new CUDACompressionTools();

      // Compress the data
      BytePointer compressedData = compressor.compress(depthImage);
      long compressedDataSize = compressedData.limit();

      // Compressed data should be smaller than original
      assertTrue(compressedDataSize <= originalDataSize);

      // Decompress the data
      BytePointer decompressedData = compressor.decompress(compressedData, compressedDataSize);
      long decompressedDataSize = decompressedData.limit();

      // Decompressed data should be same size as original
      assertEquals(originalDataSize, decompressedDataSize);
      // Lossless compression; decompressed data should be the same as original
      assertTrue(dataEquals(depthImage.data(), decompressedData, decompressedDataSize));

      // Free everything
      compressedData.close();
      decompressedData.close();
      compressor.destroy();

      depthImage.close();
   }

   @Test
   public void testGPUMatCompression() throws IOException
   {
      // Read ZED depth image data
      byte[] depthImageData = Files.readAllBytes(zedDepth16UPath);
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16U, new BytePointer(depthImageData));
      long originalDataSize = depthImage.elemSize() * depthImage.total();

      GpuMat gpuDepthImage = new GpuMat(depthImage.size(), depthImage.type());
      gpuDepthImage.upload(depthImage);

      CUDACompressionTools compressor = new CUDACompressionTools();

      // Compress the data
      BytePointer compressedData = compressor.compress(gpuDepthImage);
      long compressedDataSize = compressedData.limit();

      // Compressed data should be smaller than original
      assertTrue(compressedDataSize <= originalDataSize);

      // Decompress the data
      BytePointer decompressedData = compressor.decompress(compressedData, compressedDataSize);
      long decompressedDataSize = decompressedData.limit();

      // Decompressed data should be same size as original
      assertEquals(originalDataSize, decompressedDataSize);
      // Lossless compression; decompressed data should be the same as original
      assertTrue(dataEquals(depthImage.data(), decompressedData, decompressedDataSize));

      // Free everything
      compressedData.close();
      decompressedData.close();
      compressor.destroy();

      depthImage.close();
      gpuDepthImage.close();
   }

   private boolean dataEquals(BytePointer dataA, BytePointer dataB, long dataSize)
   {
      for (long i = 0L; i < dataSize; ++i)
      {
         if (dataA.get(i) != dataB.get(i))
            return false;
      }

      return true;
   }
}
