package us.ihmc.perception.cuda;

import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImageTest;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.io.IOException;
import java.nio.file.Files;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;

public class CUDACompressionToolsTest
{
   private static final WorkspaceResourceDirectory resourceDirectory = new WorkspaceResourceDirectory(RawImageTest.class);
   private static final WorkspaceFile zedColorBGRFile = new WorkspaceFile(resourceDirectory, "zedColorBGR.raw");
   private static final WorkspaceFile zedDepth16UFile = new WorkspaceFile(resourceDirectory, "zedDepth16U.raw");

   @Test
   public void testCPUDepthCompressionDecompression() throws IOException
   {
      byte[] depthBytes = Files.readAllBytes(zedDepth16UFile.getFilesystemFile());
      long originalDataSize = depthBytes.length;
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16UC1, new BytePointer(depthBytes));

      CUDACompressionTools compressor = new CUDACompressionTools();
      BytePointer compressedLSBData = new BytePointer();
      BytePointer compressedMSBData = new BytePointer();
      compressor.compressDepth(depthImage, compressedLSBData, compressedMSBData);

      // Sum of data size should be less than original data
      assertTrue(compressedLSBData.limit() + compressedMSBData.limit() < originalDataSize);

      Mat decompressedDepth = new Mat(depthImage.size(), depthImage.type());
      compressor.decompressDepth(compressedLSBData, compressedMSBData, compressedMSBData.limit(), decompressedDepth);

      double averageDifference = averagePixelDifference(depthImage, decompressedDepth);
      LogTools.info("Difference Ratio: {}", averageDifference);
      assertTrue(averageDifference < 1.0); // On average, decoded pixels differ less than 1mm from the original
   }

   @Test
   public void testGPUDepthCompressionDecompression() throws IOException
   {
      byte[] depthBytes = Files.readAllBytes(zedDepth16UFile.getFilesystemFile());
      long originalDataSize = depthBytes.length;
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16UC1, new BytePointer(depthBytes));
      GpuMat gpuDepthImage = new GpuMat(depthImage.size(), depthImage.type());
      gpuDepthImage.upload(depthImage);

      CUDACompressionTools compressor = new CUDACompressionTools();
      BytePointer compressedLSBData = new BytePointer(originalDataSize);
      BytePointer compressedMSBData = new BytePointer();
      compressor.compressDepth(gpuDepthImage, compressedLSBData, compressedMSBData);

      // Sum of data size should be less than original data
      assertTrue(compressedLSBData.limit() + compressedMSBData.limit() < originalDataSize);

      GpuMat decompressedDepth = new GpuMat(depthImage.size(), depthImage.type());
      compressor.decompressDepth(compressedLSBData, compressedLSBData.limit(), compressedMSBData, compressedMSBData.limit(), decompressedDepth);

      Mat cpuDecompressedDepth = new Mat(720, 1280, opencv_core.CV_16UC1, new Scalar(0.0));
      decompressedDepth.download(cpuDecompressedDepth);
      double averageDifference = averagePixelDifference(depthImage, cpuDecompressedDepth);
      LogTools.info("Difference Ratio: {}", averageDifference);
      assertTrue(averageDifference < 1.0); // On average, decoded pixels differ less than 1mm from the original

   }

   @Test
   public void testBasicCompression() throws IOException
   {
      // Read ZED color image data
      byte[] colorImageData = Files.readAllBytes(zedColorBGRFile.getFilesystemFile());
      long originalDataSize = colorImageData.length;
      BytePointer originalData = new BytePointer(colorImageData);

      CUDACompressionTools compressor = new CUDACompressionTools();

      // Compress the data
      BytePointer compressedData = new BytePointer();
      long compressedDataSize = compressor.compress(originalData, originalDataSize, compressedData);

      // Compressed data should be smaller than original
      assertTrue(compressedDataSize <= originalDataSize);

      // Decompress the data
      BytePointer decompressedData = new BytePointer();
      long decompressedDataSize = compressor.decompress(compressedData, compressedDataSize, decompressedData);

      // Decompressed data should be same size as original
      assertEquals(originalDataSize, decompressedDataSize);
      // Lossless compression; decompressed data should be the same as original
      assertTrue(dataEquals(originalData, decompressedData, decompressedDataSize));

      // Free everything
      checkCUDAError(cudart.cudaFreeHost(compressedData));
      compressedData.close();
      checkCUDAError(cudart.cudaFreeHost(decompressedData));
      decompressedData.close();
      compressor.destroy();
      originalData.close();
   }

   @Test
   public void testCPUMatCompression() throws IOException
   {
      // Read ZED depth image data
      byte[] depthImageData = Files.readAllBytes(zedDepth16UFile.getFilesystemFile());
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16U, new BytePointer(depthImageData));
      long originalDataSize = depthImage.elemSize() * depthImage.total();

      CUDACompressionTools compressor = new CUDACompressionTools();

      // Compress the data
      BytePointer compressedData = new BytePointer();
      long compressedDataSize = compressor.compress(depthImage, compressedData);

      // Compressed data should be smaller than original
      assertTrue(compressedDataSize <= originalDataSize);

      // Decompress the data
      BytePointer decompressedData = new BytePointer();
      long decompressedDataSize = compressor.decompress(compressedData, compressedDataSize, decompressedData);

      // Decompressed data should be same size as original
      assertEquals(originalDataSize, decompressedDataSize);
      // Lossless compression; decompressed data should be the same as original
      assertTrue(dataEquals(depthImage.data(), decompressedData, decompressedDataSize));

      // Free everything
      checkCUDAError(cudart.cudaFreeHost(compressedData));
      compressedData.close();
      checkCUDAError(cudart.cudaFreeHost(decompressedData));
      decompressedData.close();
      compressor.destroy();

      depthImage.close();
   }

   @Test
   public void testGPUMatCompression() throws IOException
   {
      // Read ZED depth image data
      byte[] depthImageData = Files.readAllBytes(zedDepth16UFile.getFilesystemFile());
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16U, new BytePointer(depthImageData));
      long originalDataSize = depthImage.elemSize() * depthImage.total();

      GpuMat gpuDepthImage = new GpuMat(depthImage.size(), depthImage.type());
      gpuDepthImage.upload(depthImage);

      CUDACompressionTools compressor = new CUDACompressionTools();

      // Compress the data
      BytePointer compressedData = new BytePointer();
      long compressedDataSize = compressor.compress(gpuDepthImage, compressedData);

      // Compressed data should be smaller than original
      assertTrue(compressedDataSize <= originalDataSize);

      // Decompress the data
      BytePointer decompressedData = new BytePointer();
      long decompressedDataSize = compressor.decompress(compressedData, compressedDataSize, decompressedData);

      // Decompressed data should be same size as original
      assertEquals(originalDataSize, decompressedDataSize);
      // Lossless compression; decompressed data should be the same as original
      assertTrue(dataEquals(depthImage.data(), decompressedData, decompressedDataSize));

      // Free everything
      checkCUDAError(cudart.cudaFreeHost(compressedData));
      compressedData.close();
      checkCUDAError(cudart.cudaFreeHost(decompressedData));
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

   private double averagePixelDifference(Mat matA, Mat matB)
   {
      if (!OpenCVTools.dimensionsMatch(matA, matB))
         return 1.0;

      if (OpenCVTools.dataSize(matA) != OpenCVTools.dataSize(matB))
         return 1.0;

      try (Mat differenceMat = new Mat();)
      {
         // Find absolute difference for each element
         opencv_core.absdiff(matA, matB, differenceMat);

         // Find the sum of the differences
         double totalDifference = opencv_core.sumElems(differenceMat).get();

         // Divide total difference by max difference (255 * total elements)
         double differenceRatio = totalDifference / matA.total();
         return differenceRatio;
      }
   }
}
