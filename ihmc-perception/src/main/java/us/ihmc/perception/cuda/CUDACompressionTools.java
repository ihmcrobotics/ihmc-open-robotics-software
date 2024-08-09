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
import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;

public class CUDACompressionTools
{
   private static final long CHUNK_SIZE = 1 << 16;

   private final Mat depthMSBExtractorCPU = new Mat(1, 1, opencv_core.CV_16UC1, new Scalar(65280.0));
   private final Mat depthLSBExtractorCPU = new Mat(1, 1, opencv_core.CV_16UC1, new Scalar(255.0));
   private final GpuMat depthMSBExtractorGPU = new GpuMat(1, 1, opencv_core.CV_16UC1);
   private final GpuMat depthLSBExtractorGPU = new GpuMat(1, 1, opencv_core.CV_16UC1);

   private final CUstream_st stream;

   private final PimplManager compressionManager;
   private final nvcompBatchedZstdOpts_t zstdOptions;

   private final CUDAJPEGProcessor jpegEncoder = new CUDAJPEGProcessor();

   CUDAJPEGProcessor lossyImageEncoder = new CUDAJPEGProcessor();

   public CUDACompressionTools()
   {
      stream = CUDAStreamManager.getStream();

      zstdOptions = nvcomp.nvcompBatchedZstdDefaultOpts();
      compressionManager = new ZstdManager(CHUNK_SIZE, zstdOptions, stream, 0, nvcomp.NoComputeNoVerify);

      depthMSBExtractorGPU.upload(depthMSBExtractorCPU);
      depthLSBExtractorGPU.upload(depthLSBExtractorCPU );
   }

   public void destroy()
   {
      checkCUDAError(cudaStreamSynchronize(stream));
      CUDAStreamManager.releaseStream(stream);
      compressionManager.close();
      lossyImageEncoder.destroy();

      depthMSBExtractorCPU.close();
      depthLSBExtractorCPU.close();
      depthMSBExtractorGPU.close();
      depthLSBExtractorGPU.close();
   }

   public long compress(Mat mat, BytePointer compressedData)
   {
      return compress(mat.data(), OpenCVTools.dataSize(mat), compressedData);
   }

   public long compress(GpuMat gpuMat, BytePointer compressedData)
   {
      long rowSize = gpuMat.elemSize() * gpuMat.cols();

      BytePointer gpuMatData = new BytePointer();
      checkCUDAError(cudaMallocAsync(gpuMatData, OpenCVTools.dataSize(gpuMat), stream));
      checkCUDAError(cudaMemcpy2DAsync(gpuMatData, rowSize, gpuMat.data(), gpuMat.step(), rowSize, gpuMat.rows(), cudaMemcpyDefault, stream));

      long compressedSize = compress(gpuMatData, OpenCVTools.dataSize(gpuMat), compressedData);

      checkCUDAError(cudaFreeAsync(gpuMatData, stream));
      gpuMatData.close();

      return compressedSize;
   }

   /**
    * Compresses data using the provided compression manager.
    * @param data INPUT: Pointer to data to be compressed (can point to host or device memory)
    * @param dataSize INPUT: Size of data to compress, in bytes
    * @param compressedData OUTPUT: Pointer to compressed data (must be a host pointer). MUST BE FREED USING {@link cudart#cudaFreeHost(Pointer)}.
    * @return Size of compressed data, in bytes.
    */
   public long compress(BytePointer data, long dataSize, BytePointer compressedData)
   {
      return compress(data, dataSize, compressedData, compressionManager, stream);
   }

   /**
    * Decompresses data using a new compression manager.
    * @param compressedData INPUT: Pointer to compressed data (can point to host or device memory)
    * @param compressedDataSize INPUT: Size of the compressed data, in bytes
    * @param decompressedData OUTPUT: Pointer to decompressed data (must be a host pointer). MUST BE FREED USING {@link cudart#cudaFreeHost(Pointer)}.
    * @return Size of decompressed data, in bytes.
    */
   public long decompress(BytePointer compressedData, long compressedDataSize, BytePointer decompressedData)
   {
      return decompress(compressedData, compressedDataSize, decompressedData, stream);
   }

   public void compressDepth(Mat depthImage, BytePointer compressedLSBData, BytePointer compressedMSBData)
   {
      try (Mat depthLSB = new Mat(depthImage.size(), opencv_core.CV_16UC1);
           Mat depthMSB = new Mat(depthImage.size(), opencv_core.CV_16UC1))
      {
         // Ensure extractor dimensions match
         if (!OpenCVTools.dimensionsMatch(depthImage, depthLSBExtractorCPU))
            opencv_imgproc.resize(depthLSBExtractorCPU, depthLSBExtractorCPU, depthImage.size());
         if (!OpenCVTools.dimensionsMatch(depthImage, depthMSBExtractorCPU))
            opencv_imgproc.resize(depthMSBExtractorCPU, depthMSBExtractorCPU, depthImage.size());

         // Bitwise and to leave only wanted bits
         opencv_core.bitwise_and(depthImage, depthLSBExtractorCPU, depthLSB);
         opencv_core.bitwise_and(depthImage, depthMSBExtractorCPU, depthMSB);

         // Convert to 8 bit type
         depthLSB.convertTo(depthLSB, opencv_core.CV_8UC1);
         depthMSB.convertTo(depthMSB, opencv_core.CV_8UC1, 1.0 / 256.0, 0.0);

         // Use lossless compression for MSB
         compress(depthMSB, compressedMSBData);

         // Use lossy compression for LSB
         opencv_imgcodecs.imencode(".jpg", depthLSB, compressedLSBData);

         checkCUDAError(cudaFreeHost(compressedMSBData));
      }
   }

   public void compressDepth(GpuMat depthImage, BytePointer compressedLSBData, BytePointer compressedMSBData)
   {
      try (GpuMat depthLSB = new GpuMat(depthImage.size(), opencv_core.CV_16UC1);
           GpuMat depthMSB = new GpuMat(depthImage.size(), opencv_core.CV_16UC1))
      {
         // Ensure extractor dimensions match
         if (!OpenCVTools.dimensionsMatch(depthLSBExtractorCPU, depthImage))
         {
            opencv_imgproc.resize(depthLSBExtractorCPU, depthLSBExtractorCPU, depthImage.size());
            depthLSBExtractorGPU.upload(depthLSBExtractorCPU);
         }
         if (!OpenCVTools.dimensionsMatch(depthMSBExtractorCPU, depthImage))
         {
            opencv_imgproc.resize(depthMSBExtractorCPU, depthMSBExtractorCPU, depthImage.size());
            depthMSBExtractorGPU.upload(depthMSBExtractorCPU);
         }

         opencv_cudaarithm.bitwise_and(depthImage, depthLSBExtractorGPU, depthLSB);
         opencv_cudaarithm.bitwise_and(depthImage, depthMSBExtractorGPU, depthMSB);

         depthLSB.convertTo(depthLSB, opencv_core.CV_8UC1);
         depthMSB.convertTo(depthMSB, opencv_core.CV_8UC1, 1.0 / 256.0, 0.0);

         compress(depthMSB, compressedMSBData);
         jpegEncoder.encodeGray(depthLSB, compressedLSBData);
         checkCUDAError(cudaFreeHost(compressedMSBData));
      }
   }

   public void decompressDepth(BytePointer compressedLSBData, BytePointer compressedMSBData, long msbDataSize, Mat decompressedDepth)
   {
      try (BytePointer decompressedMSBData = new BytePointer();
           Mat compressedLSBMat = new Mat(compressedLSBData);
           Mat decompressedLSBMat = new Mat(decompressedDepth.size(), opencv_core.CV_8UC1))
      {
         opencv_imgcodecs.imdecode(compressedLSBMat, opencv_imgcodecs.IMREAD_UNCHANGED, decompressedLSBMat);
         decompress(compressedMSBData, msbDataSize, decompressedMSBData);

         Mat decompressedMSBMat = new Mat(decompressedDepth.size(), opencv_core.CV_8UC1, decompressedMSBData);
         cudaFreeHost(decompressedMSBData);

         decompressedLSBMat.convertTo(decompressedLSBMat, opencv_core.CV_16UC1);
         decompressedMSBMat.convertTo(decompressedMSBMat, opencv_core.CV_16UC1, 256.0, 0.0);

         opencv_core.bitwise_or(decompressedMSBMat, decompressedLSBMat, decompressedDepth);

         decompressedMSBMat.close();
      }
   }

   public void decompressDepth(BytePointer compressedLSBData, long lsbDataSize, BytePointer compressedMSBData, long msbDataSize, GpuMat decompressedDepth)
   {
      try (BytePointer decompressedMSBData = new BytePointer();
           GpuMat decompressedLSBMat = new GpuMat(decompressedDepth.size(), opencv_core.CV_8UC1))
      {
         decompress(compressedMSBData, msbDataSize, decompressedMSBData);
         jpegEncoder.decodeToGray(compressedLSBData, lsbDataSize, decompressedLSBMat);

         GpuMat decompressedMSBMat = new GpuMat(decompressedDepth.size(), opencv_core.CV_8UC1, decompressedMSBData);
         cudaFreeHost(decompressedMSBData);

         decompressedLSBMat.convertTo(decompressedLSBMat, opencv_core.CV_16UC1);
         decompressedMSBMat.convertTo(decompressedMSBMat, opencv_core.CV_16UC1, 256.0, 0.0);

         opencv_cudaarithm.bitwise_or(decompressedMSBMat, decompressedLSBMat, decompressedDepth);

         decompressedMSBMat.close();
      }
   }

   /**
    * Compresses data using the provided compression manager.
    * @param data INPUT: Pointer to data to be compressed (can point to host or device memory, must be continuous)
    * @param dataSize INPUT: Size of data to compress, in bytes
    * @param compressedData OUTPUT: Pointer to compressed data (must be a host pointer). MUST BE FREED USING {@link cudart#cudaFreeHost(Pointer)}.
    * @param compressionManager INPUT: Compression manager to use
    * @param cudaStream INPUT: CUDA stream to use
    * @return Size of compressed data, in bytes.
    */
   public static long compress(BytePointer data, long dataSize, BytePointer compressedData, PimplManager compressionManager, CUstream_st cudaStream)
   {
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
         compressedData.limit(compressedDataSize);
         checkCUDAError(cudaMallocHost(compressedData, compressedDataSize));
         checkCUDAError(cudaMemcpyAsync(compressedData, compressedDeviceBuffer, compressedDataSize, cudart.cudaMemcpyDeviceToHost, cudaStream));
         checkCUDAError(cudaStreamSynchronize(cudaStream));

         // Free GPU memory
         checkCUDAError(cudaFreeAsync(compressedDeviceBuffer, cudaStream));
         checkCUDAError(cudaFreeAsync(uncompressedDeviceBuffer, cudaStream));

         return compressedDataSize;
      }
   }

   /**
    * Decompresses data using a new compression manager.
    * @param compressedData INPUT: Pointer to compressed data (can point to host or device memory)
    * @param compressedDataSize INPUT: Size of the compressed data, in bytes
    * @param decompressedData OUTPUT: Pointer to decompressed data (must be a host pointer). MUST BE FREED USING {@link cudart#cudaFreeHost(Pointer)}.
    * @param cudaStream INPUT: CUDA stream to use
    * @return Size of decompressed data, in bytes.
    */
   public static long decompress(BytePointer compressedData, long compressedDataSize, BytePointer decompressedData, CUstream_st cudaStream)
   {
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

         // Download decompressed data to host
         decompressedData.limit(decompressedDataSize);
         checkCUDAError(cudaMallocHost(decompressedData, decompressedDataSize));
         checkCUDAError(cudaMemcpyAsync(decompressedData, decompressedDeviceBuffer, decompressedDataSize, cudart.cudaMemcpyDeviceToHost, cudaStream));
         checkCUDAError(cudaStreamSynchronize(cudaStream));

         checkCUDAError(cudaFreeAsync(compressedDeviceBuffer, cudaStream));
         checkCUDAError(cudaFreeAsync(decompressedDeviceBuffer, cudaStream));

         return decompressedDataSize;
      }
   }
}
