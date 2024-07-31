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

import static org.bytedeco.cuda.global.cudart.*;
import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;

public class CUDACompressionTools
{
   private static final long CHUNK_SIZE = 1 << 16;

   private final CUstream_st stream;

   private final PimplManager compressionManager;
   private final nvcompBatchedZstdOpts_t zstdOptions;

   public CUDACompressionTools()
   {
      stream = CUDAStream.getStream();

      zstdOptions = nvcomp.nvcompBatchedZstdDefaultOpts();
      compressionManager = new ZstdManager(CHUNK_SIZE, zstdOptions, stream, 0, nvcomp.NoComputeNoVerify);
   }

   public void destroy()
   {
      CUDAStream.releaseStream(stream);
      compressionManager.close();
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

   /**
    * Compresses data using the provided compression manager.
    * @param data INPUT: Pointer to data to be compressed (can point to host or device memory)
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
         checkCUDAError(cudaMallocHost(compressedData, compressedDataSize));
         checkCUDAError(cudaMemcpyAsync(compressedData, compressedDeviceBuffer, compressedDataSize, cudart.cudaMemcpyDeviceToHost, cudaStream));

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
         checkCUDAError(cudaMallocHost(decompressedData, decompressedDataSize));
         checkCUDAError(cudaMemcpyAsync(decompressedData, decompressedDeviceBuffer, decompressedDataSize, cudart.cudaMemcpyDeviceToHost, cudaStream));

         checkCUDAError(cudaFreeAsync(compressedDeviceBuffer, cudaStream));
         checkCUDAError(cudaFreeAsync(decompressedDeviceBuffer, cudaStream));

         return decompressedDataSize;
      }
   }
}
