package us.ihmc.perception.cuda;

import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.RawImageTest;
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
   public void testBasicCompression() throws IOException
   {
      // Read ZED color image data
      byte[] colorImageData = Files.readAllBytes(zedColorBGRFile.getFilesystemFile());
      long originalDataSize = colorImageData.length;
      BytePointer originalData = new BytePointer(colorImageData);

      testCPUCompression(originalData, originalDataSize);

      originalData.close();
   }

   @Test
   public void testCPUMatCompression() throws IOException
   {
      // Read ZED depth image data
      byte[] depthImageData = Files.readAllBytes(zedDepth16UFile.getFilesystemFile());
      long originalDataSize = depthImageData.length;
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16U, new BytePointer(depthImageData));

      // Test compression on the data
      testCPUCompression(depthImage.data(), originalDataSize);

      depthImage.close();
   }

   @Test
   public void testGPUMatCompression() throws IOException
   {
      // Read ZED depth image data
      byte[] depthImageData = Files.readAllBytes(zedDepth16UFile.getFilesystemFile());
      long originalDataSize = depthImageData.length;
      Mat depthImage = new Mat(720, 1280, opencv_core.CV_16U, new BytePointer(depthImageData));
      GpuMat gpuDepthImage = new GpuMat(depthImage.size(), depthImage.type());
      gpuDepthImage.upload(depthImage);

      // Test compression on GPU data
      testGPUCompression(gpuDepthImage.data(), depthImage.data(), originalDataSize);

      depthImage.close();
      gpuDepthImage.close();
   }

   private void testCPUCompression(BytePointer originalData, long originalDataSize)
   {
      CUDACompressionTools compressionTools = new CUDACompressionTools();

      // Compress the data
      BytePointer compressedData = new BytePointer();
      long compressedDataSize = compressionTools.compress(originalData, originalDataSize, compressedData);

      // Compressed data should be smaller than original
      assertTrue(compressedDataSize <= originalDataSize);

      // Decompress the data
      BytePointer decompressedData = new BytePointer();
      long decompressedDataSize = compressionTools.decompress(compressedData, compressedDataSize, decompressedData);

      // Decompressed data should be same size as original
      assertEquals(originalDataSize, decompressedDataSize);
      // Lossless compression; decompressed data should be the same as original
      assertTrue(dataEquals(originalData, decompressedData, decompressedDataSize));

      // Free everything
      checkCUDAError(cudart.cudaFreeHost(compressedData));
      compressedData.close();
      checkCUDAError(cudart.cudaFreeHost(decompressedData));
      decompressedData.close();
      compressionTools.destroy();
   }

   private void testGPUCompression(BytePointer originalGPUData, BytePointer originalCPUData, long originalDataSize)
   {
      CUDACompressionTools compressionTools = new CUDACompressionTools();

      // Compress the data
      BytePointer compressedData = new BytePointer();
      long compressedDataSize = compressionTools.compress(originalGPUData, originalDataSize, compressedData);

      // Compressed data should be smaller than original
      assertTrue(compressedDataSize <= originalDataSize);

      // Decompress the data
      BytePointer decompressedData = new BytePointer();
      long decompressedDataSize = compressionTools.decompress(compressedData, compressedDataSize, decompressedData);

      // Decompressed data should be same size as original
      assertEquals(originalDataSize, decompressedDataSize);
      // Lossless compression; decompressed data should be the same as original
      assertTrue(dataEquals(originalCPUData, decompressedData, decompressedDataSize));

      // Free everything
      checkCUDAError(cudart.cudaFreeHost(compressedData));
      compressedData.close();
      checkCUDAError(cudart.cudaFreeHost(decompressedData));
      decompressedData.close();
      compressionTools.destroy();
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
