package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.cuda.global.nvcomp;
import org.bytedeco.cuda.nvcomp.CompressionConfig;
import org.bytedeco.cuda.nvcomp.DecompressionConfig;
import org.bytedeco.cuda.nvcomp.PimplManager;
import org.bytedeco.cuda.nvcomp.ZstdManager;
import org.bytedeco.cuda.nvcomp.nvcompBatchedZstdOpts_t;
import org.bytedeco.cuda.nvcomp.nvcompManagerBase;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaarithm;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.perception.opencv.OpenCVTools;

import static org.bytedeco.cuda.global.cudart.*;
import static us.ihmc.perception.cuda.CUDATools.*;

public class CUDACompressionTools
{
   private static final boolean NVCOMP_AVAILABLE = hasCUDADevice() && hasNVCOMP();
   private static final long CHUNK_SIZE = 1 << 16;

   private final Mat depthMSBExtractorCPU = new Mat(1, 1, opencv_core.CV_16UC1, new Scalar(65280.0));
   private final Mat depthLSBExtractorCPU = new Mat(1, 1, opencv_core.CV_16UC1, new Scalar(255.0));
   private final GpuMat depthMSBExtractorGPU = new GpuMat(1, 1, opencv_core.CV_16UC1);
   private final GpuMat depthLSBExtractorGPU = new GpuMat(1, 1, opencv_core.CV_16UC1);

   private final CUstream_st stream;

   private final PimplManager compressionManager;

   private final CUDAJPEGProcessor jpegProcessor = new CUDAJPEGProcessor(40);

   public CUDACompressionTools()
   {
      if (!hasNVCOMP())
         throw new UnsatisfiedLinkError("NVCOMP was not found. To install NVCOMP, see instructions in ihmc-perception/README.md");

      stream = CUDAStreamManager.getStream();

      nvcompBatchedZstdOpts_t zstdOptions = nvcomp.nvcompBatchedZstdDefaultOpts();
      compressionManager = new ZstdManager(CHUNK_SIZE, zstdOptions, stream, 0, nvcomp.NoComputeNoVerify);

      depthMSBExtractorGPU.upload(depthMSBExtractorCPU);
      depthLSBExtractorGPU.upload(depthLSBExtractorCPU );
   }

   public void destroy()
   {
      checkCUDAError(cudaStreamSynchronize(stream));
      CUDAStreamManager.releaseStream(stream);
      compressionManager.close();
      jpegProcessor.destroy();

      depthMSBExtractorCPU.close();
      depthLSBExtractorCPU.close();
      depthMSBExtractorGPU.close();
      depthLSBExtractorGPU.close();
   }

   public BytePointer compress(Mat mat)
   {
      return compress(mat.data(), OpenCVTools.dataSize(mat));
   }

   public BytePointer compress(GpuMat gpuMat)
   {
      long rowSize = gpuMat.elemSize() * gpuMat.cols();

      BytePointer gpuMatData = new BytePointer();
      checkCUDAError(cudaMallocAsync(gpuMatData, OpenCVTools.dataSize(gpuMat), stream));
      checkCUDAError(cudaMemcpy2DAsync(gpuMatData, rowSize, gpuMat.data(), gpuMat.step(), rowSize, gpuMat.rows(), cudaMemcpyDefault, stream));

      BytePointer compressedData = compress(gpuMatData, OpenCVTools.dataSize(gpuMat));

      checkCUDAError(cudaFreeAsync(gpuMatData, stream));
      gpuMatData.close();

      return compressedData;
   }

   /**
    * Compresses data using a lossless method
    * @param data Pointer to data to be compressed (can point to host or device memory)
    * @param dataSize Size of data to compress, in bytes
    * @return BytePointer to compressed data, with limit set as the size of the compressed data.
    */
   public BytePointer compress(BytePointer data, long dataSize)
   {
      return compress(data, dataSize, compressionManager, stream);
   }

   /**
    * Decompress data which was compressed using nvCOMP
    * @param compressedData Pointer to compressed data (can point to host or device memory)
    * @param compressedDataSize Size of the compressed data, in bytes
    * @return BytePointer to decompressed data, with limit set as the size of the decompressed data
    */
   public BytePointer decompress(BytePointer compressedData, long compressedDataSize)
   {
      return decompress(compressedData, compressedDataSize, false, stream);
   }

   /**
    * Compresses depth using a half lossy, half lossless method.
    * Given a 16 bit depth image, the 8 most significant bits are compressed using a lossless method
    * and the 8 least significant bits are compressed using a lossy method.
    * This allows for a higher compression ratio, while maintaining a reasonably high quality of depth data.
    * @param depthImage The depth image to compress
    * @return BytePointer to compressed depth data.
    */
   public BytePointer compressDepth(Mat depthImage)
   {
      if (!NVCOMP_AVAILABLE)
         return new BytePointer(1);

      try (Mat depthLSB = extractDepthLSB(depthImage);
           Mat depthMSB = extractDepthMSB(depthImage);
           BytePointer compressedMSBData = compress(depthMSB);
           BytePointer compressedLSBData = new BytePointer())
      {
         opencv_imgcodecs.imencode(".jpg", depthLSB, compressedLSBData);

         return packCompressedDepth(compressedMSBData, compressedLSBData);
      }
   }

   /**
    * Compresses depth using a half lossy, half lossless method.
    * Given a 16 bit depth image, the 8 most significant bits are compressed using a lossless method
    * and the 8 least significant bits are compressed using a lossy method.
    * This allows for a higher compression ratio, while maintaining a reasonably high quality of depth data.
    * @param depthImage The depth image to compress
    * @return BytePointer to compressed depth data.
    */
   public BytePointer compressDepth(GpuMat depthImage)
   {
      if (!NVCOMP_AVAILABLE)
         return new BytePointer(1);

      try (GpuMat depthLSB = extractDepthLSB(depthImage);
           GpuMat depthMSB = extractDepthMSB(depthImage);
           BytePointer compressedMSBData = compress(depthMSB);
           BytePointer compressedLSBData = new BytePointer(OpenCVTools.dataSize(depthLSB)))
      {
         jpegProcessor.encodeGray(depthLSB, compressedLSBData);

         return packCompressedDepth(compressedMSBData, compressedLSBData);
      }
   }

   /**
    * Given compressed depth data acquired through {@link #compressDepth(Mat)}, decompresses the data into a depth image
    * @param compressedDepthData Compressed depth data acquired through {@link #compressDepth(Mat)}.
    * @param decompressedDepth Mat into which decompressed depth is packed. Must be allocated with correct dimensions.
    */
   public void decompressDepth(BytePointer compressedDepthData, Mat decompressedDepth)
   {
      GpuMat decompressedDepthGPU = new GpuMat(decompressedDepth.size(), opencv_core.CV_16UC1);
      decompressDepth(compressedDepthData, decompressedDepthGPU);
      decompressedDepthGPU.download(decompressedDepth);
      decompressedDepthGPU.close();
   }

   /**
    * Given compressed depth data acquired through {@link #compressDepth(Mat)}, decompresses the data into a depth image
    * @param compressedDepthData Compressed depth data acquired through {@link #compressDepth(Mat)}.
    * @param decompressedDepth GpuMat into which decompressed depth is packed. Must be allocated with correct dimensions.
    */
   public void decompressDepth(BytePointer compressedDepthData, GpuMat decompressedDepth)
   {
      if (!NVCOMP_AVAILABLE)
         return;

      BytePointer[] depthData = unpackCompressedDepth(compressedDepthData);

      try (BytePointer decompressedMSBData = decompress(depthData[0], depthData[0].limit(), true, stream);
           GpuMat decompressedMSBMat = new GpuMat(decompressedDepth.size(), opencv_core.CV_8UC1, decompressedMSBData);
           GpuMat decompressedLSBMat = new GpuMat(decompressedDepth.size(), opencv_core.CV_8UC1))
      {
         jpegProcessor.decodeToGray(depthData[1], depthData[1].limit(), decompressedLSBMat);

         decompressedLSBMat.convertTo(decompressedLSBMat, opencv_core.CV_16UC1);
         decompressedMSBMat.convertTo(decompressedMSBMat, opencv_core.CV_16UC1, 256.0, 0.0);

         opencv_cudaarithm.bitwise_or(decompressedMSBMat, decompressedLSBMat, decompressedDepth);

         checkCUDAError(cudaFreeAsync(decompressedMSBData, stream));
      }

      for (BytePointer data : depthData)
         data.close();
   }

   private Mat extractDepthLSB(Mat depthImage)
   {
      if (!OpenCVTools.dimensionsMatch(depthLSBExtractorCPU, depthImage))
         opencv_imgproc.resize(depthLSBExtractorCPU, depthLSBExtractorCPU, depthImage.size());

      Mat depthLSB = new Mat();
      opencv_core.bitwise_and(depthImage, depthLSBExtractorCPU, depthLSB);
      depthLSB.convertTo(depthLSB, opencv_core.CV_8UC1);
      return depthLSB;
   }

   private GpuMat extractDepthLSB(GpuMat depthImage)
   {
      // Ensure extractor dimensions match
      if (!OpenCVTools.dimensionsMatch(depthLSBExtractorCPU, depthImage))
      {
         opencv_imgproc.resize(depthLSBExtractorCPU, depthLSBExtractorCPU, depthImage.size());
         depthLSBExtractorGPU.upload(depthLSBExtractorCPU);
      }

      GpuMat depthLSB = new GpuMat();
      opencv_cudaarithm.bitwise_and(depthImage, depthLSBExtractorGPU, depthLSB);
      depthLSB.convertTo(depthLSB, opencv_core.CV_8UC1);
      return depthLSB;
   }

   private Mat extractDepthMSB(Mat depthImage)
   {
      if (!OpenCVTools.dimensionsMatch(depthMSBExtractorCPU, depthImage))
         opencv_imgproc.resize(depthMSBExtractorCPU, depthMSBExtractorCPU, depthImage.size());

      Mat depthMSB = new Mat();
      opencv_core.bitwise_and(depthImage, depthMSBExtractorCPU, depthMSB);
      depthMSB.convertTo(depthMSB, opencv_core.CV_8UC1, 1.0 / 256.0, 0.0);
      return depthMSB;
   }

   private GpuMat extractDepthMSB(GpuMat depthImage)
   {
      if (!OpenCVTools.dimensionsMatch(depthMSBExtractorCPU, depthImage))
      {
         opencv_imgproc.resize(depthMSBExtractorCPU, depthMSBExtractorCPU, depthImage.size());
         depthMSBExtractorGPU.upload(depthMSBExtractorCPU);
      }

      GpuMat depthMSB = new GpuMat();
      opencv_cudaarithm.bitwise_and(depthImage, depthMSBExtractorGPU, depthMSB);
      depthMSB.convertTo(depthMSB, opencv_core.CV_8UC1, 1.0 / 256.0, 0.0);
      return depthMSB;
   }

   private BytePointer packCompressedDepth(BytePointer depthMSB, BytePointer depthLSB)
   {
      long dataSize = 2 * Long.BYTES + depthMSB.limit() + depthLSB.limit();
      BytePointer packedBuffer = new BytePointer(dataSize);
      long offset = 0;
      packedBuffer.putLong(depthMSB.limit());
      offset += Long.BYTES;

      packedBuffer.position(offset);
      packedBuffer.putLong(depthLSB.limit());
      offset += Long.BYTES;

      packedBuffer.position(offset);
      BytePointer.memcpy(packedBuffer, depthMSB, depthMSB.limit());
      offset += depthMSB.limit();

      packedBuffer.position(offset);
      BytePointer.memcpy(packedBuffer, depthLSB, depthLSB.limit());

      packedBuffer.position(0);
      packedBuffer.limit(dataSize);

      return packedBuffer;
   }

   private BytePointer[] unpackCompressedDepth(BytePointer packedBuffer)
   {
      long offset = 0;
      packedBuffer.position(offset);
      long msbDataSize = packedBuffer.getLong();
      offset += Long.BYTES;

      packedBuffer.position(offset);
      long lsbDataSize = packedBuffer.getLong();
      offset += Long.BYTES;

      BytePointer[] unpackedData = new BytePointer[2];
      unpackedData[0] = new BytePointer(msbDataSize);
      unpackedData[1] = new BytePointer(lsbDataSize);

      packedBuffer.position(offset);
      BytePointer.memcpy(unpackedData[0], packedBuffer, msbDataSize);
      offset += msbDataSize;

      packedBuffer.position(offset);
      BytePointer.memcpy(unpackedData[1], packedBuffer, lsbDataSize);

      return unpackedData;
   }

   /**
    * Compresses data using the provided compression manager.
    * @param data Pointer to data to be compressed (can point to host or device memory, must be continuous)
    * @param dataSize Size of data to compress, in bytes
    * @param compressionManager INPUT: Compression manager to use
    * @param cudaStream CUDA stream to use
    * @return BytePointer to compressed data, with limit set as the size of the compressed data.
    */
   public static BytePointer compress(BytePointer data, long dataSize, PimplManager compressionManager, CUstream_st cudaStream)
   {
      if (!NVCOMP_AVAILABLE)
         return new BytePointer(1);

      try (BytePointer uncompressedDeviceBuffer = new BytePointer();
           BytePointer compressedDeviceBuffer = new BytePointer();
           CompressionConfig compressionConfig = compressionManager.configure_compression(dataSize))
      {
         // Upload data to GPU
         checkCUDAError(cudaMallocAsync(uncompressedDeviceBuffer, dataSize, cudaStream));
         checkCUDAError(cudaMemcpyAsync(uncompressedDeviceBuffer, data, dataSize, cudart.cudaMemcpyDefault, cudaStream));

         // Allocate GPU memory for compressed data
         checkCUDAError(cudaMallocAsync(compressedDeviceBuffer, compressionConfig.max_compressed_buffer_size(), cudaStream));

         // Compress the data
         compressionManager.compress(uncompressedDeviceBuffer, compressedDeviceBuffer, compressionConfig);
         long compressedDataSize = compressionManager.get_compressed_output_size(compressedDeviceBuffer);

         // Download compressed data to host
         BytePointer compressedData = new BytePointer(compressedDataSize);
         compressedData.limit(compressedDataSize);
         checkCUDAError(cudaMemcpyAsync(compressedData, compressedDeviceBuffer, compressedDataSize, cudart.cudaMemcpyDeviceToHost, cudaStream));
         checkCUDAError(cudaStreamSynchronize(cudaStream));

         // Free GPU memory
         checkCUDAError(cudaFreeAsync(compressedDeviceBuffer, cudaStream));
         checkCUDAError(cudaFreeAsync(uncompressedDeviceBuffer, cudaStream));

         return compressedData;
      }
   }

   /**
    * Decompresses data using a new compression manager.
    * @param compressedData Pointer to compressed data (can point to host or device memory)
    * @param compressedDataSize Size of the compressed data, in bytes
    * @param outputToDevice Whether to output decompressed data to device (MUST CALL {@link cudart#cudaFree(Pointer)} ON OUTPUT IF SET TO {@code true})
    * @param cudaStream CUDA stream to use
    * @return BytePointer to decompressed data, either on host or device memory depending on the {@code outputToDevice} parameter.
    */
   public static BytePointer decompress(BytePointer compressedData, long compressedDataSize, boolean outputToDevice, CUstream_st cudaStream)
   {
      if (!NVCOMP_AVAILABLE)
         return new BytePointer(1);

      try (BytePointer compressedDeviceBuffer = new BytePointer();
           BytePointer decompressedDeviceBuffer = new BytePointer())
      {
         // Upload compressed data to GPU
         checkCUDAError(cudaMallocAsync(compressedDeviceBuffer, compressedDataSize, cudaStream));
         checkCUDAError(cudaMemcpyAsync(compressedDeviceBuffer, compressedData, compressedDataSize, cudart.cudaMemcpyDefault, cudaStream));

         // Create a decompression manager
         nvcompManagerBase decompressionManager = nvcomp.create_manager(compressedDeviceBuffer, cudaStream, 0, nvcomp.NoComputeNoVerify);
         DecompressionConfig decompressionConfig = decompressionManager.configure_decompression(compressedDeviceBuffer);
         long decompressedDataSize = decompressionConfig.decomp_data_size();

         // Allocate GPU memory for decompressed data
         checkCUDAError(cudaMallocAsync(decompressedDeviceBuffer, decompressedDataSize, cudaStream));

         // Decompress the data
         decompressionManager.decompress(decompressedDeviceBuffer, compressedDeviceBuffer, decompressionConfig);

         BytePointer decompressedData;
         if (outputToDevice) // Copy data to output pointer on device
         {
            decompressedData = new BytePointer();
            cudaMallocAsync(decompressedData, decompressedDataSize, cudaStream);
         }
         else // Download decompressed data to host
         {
            decompressedData = new BytePointer(decompressedDataSize);
            decompressedData.limit(decompressedDataSize);
         }
         checkCUDAError(cudaMemcpyAsync(decompressedData, decompressedDeviceBuffer, decompressedDataSize, cudaMemcpyDefault, cudaStream));
         checkCUDAError(cudaStreamSynchronize(cudaStream));

         checkCUDAError(cudaFreeAsync(compressedDeviceBuffer, cudaStream));
         checkCUDAError(cudaFreeAsync(decompressedDeviceBuffer, cudaStream));
         decompressionManager.close();
         decompressionConfig.close();

         return decompressedData;
      }
   }
}
