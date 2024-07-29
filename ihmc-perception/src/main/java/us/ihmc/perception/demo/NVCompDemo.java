package us.ihmc.perception.demo;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.global.nvcomp;
import org.bytedeco.cuda.nvcomp.ANSManager;
import org.bytedeco.cuda.nvcomp.BitcompManager;
import org.bytedeco.cuda.nvcomp.CompressionConfig;
import org.bytedeco.cuda.nvcomp.DecompressionConfig;
import org.bytedeco.cuda.nvcomp.DeflateManager;
import org.bytedeco.cuda.nvcomp.GdeflateManager;
import org.bytedeco.cuda.nvcomp.LZ4Manager;
import org.bytedeco.cuda.nvcomp.PimplManager;
import org.bytedeco.cuda.nvcomp.SnappyManager;
import org.bytedeco.cuda.nvcomp.ZstdManager;
import org.bytedeco.cuda.nvcomp.nvcompBatchedANSOpts_t;
import org.bytedeco.cuda.nvcomp.nvcompBatchedBitcompFormatOpts;
import org.bytedeco.cuda.nvcomp.nvcompBatchedDeflateOpts_t;
import org.bytedeco.cuda.nvcomp.nvcompBatchedGdeflateOpts_t;
import org.bytedeco.cuda.nvcomp.nvcompBatchedLZ4Opts_t;
import org.bytedeco.cuda.nvcomp.nvcompBatchedSnappyOpts_t;
import org.bytedeco.cuda.nvcomp.nvcompBatchedZstdOpts_t;
import org.bytedeco.cuda.nvcomp.nvcompManagerBase;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

import static org.bytedeco.cuda.global.cudart.*;
import static org.bytedeco.cuda.global.nvcomp.*;

public class NVCompDemo
{
   private static final String FILE_PATH = "/home/tbialek/Documents/ZED/ZED_ColorImage.png";
   private static final int DATA_TYPE = NVCOMP_TYPE_CHAR;

   private final CUstream_st stream;
   private final long chunkSize = 1 << 16;

   // LZ4
   private final LZ4Manager lz4Manager;
   private final nvcompBatchedLZ4Opts_t lz4Options;

   // Snappy
   private final SnappyManager snappyManager;
   private final nvcompBatchedSnappyOpts_t snappyOptions;

   // Deflate
   private final DeflateManager deflateManager;
   private final nvcompBatchedDeflateOpts_t deflateOptions;

   // GDeflate
   private final GdeflateManager gDeflateManager;
   private final nvcompBatchedGdeflateOpts_t gDeflateOptions;

   // ZSTD
   private final ZstdManager zstdManager;
   private final nvcompBatchedZstdOpts_t zstdOptions;

   // ANS
   private final ANSManager ansManager;
   private final nvcompBatchedANSOpts_t ansOptions;

   // Bitcomp
   private final BitcompManager bitcompManager;
   private final nvcompBatchedBitcompFormatOpts bitcompOptions;

   private final Map<String, PimplManager> compressionManagers = new TreeMap<>();

   private NVCompDemo()
   {
      System.out.println("Chunk Size: " + chunkSize);

      stream = new CUstream_st();
      checkError(cudaStreamCreate(stream));

      // LZ4
      lz4Options = new nvcompBatchedLZ4Opts_t();
      lz4Options.data_type(DATA_TYPE);
      lz4Manager = new LZ4Manager(chunkSize, lz4Options, stream, 0, nvcomp.NoComputeNoVerify);
      compressionManagers.put("LZ4", lz4Manager);

      // Snappy
      snappyOptions = nvcomp.nvcompBatchedSnappyDefaultOpts(); // TODO: Find out what the "reserved" option does
      snappyManager = new SnappyManager(chunkSize, snappyOptions, stream, 0, NoComputeNoVerify);
      compressionManagers.put("Snappy", snappyManager);

      // Deflate
      deflateOptions = nvcomp.nvcompBatchedDeflateDefaultOpts();
      deflateManager = new DeflateManager(chunkSize, deflateOptions, stream, 0, NoComputeNoVerify);
      compressionManagers.put("Deflate", deflateManager);

      // GDeflate
      gDeflateOptions = nvcompBatchedGdeflateDefaultOpts();
      gDeflateManager = new GdeflateManager(chunkSize, gDeflateOptions, stream, 0, NoComputeNoVerify);
      compressionManagers.put("GDeflate", gDeflateManager);

      // ZSTD
      zstdOptions = nvcomp.nvcompBatchedZstdDefaultOpts();
      zstdManager = new ZstdManager(chunkSize, zstdOptions, stream, 0, NoComputeNoVerify);
      compressionManagers.put("ZSTD", zstdManager);

      // ANS
      ansOptions = nvcompBatchedANSDefaultOpts();
      ansManager = new ANSManager(chunkSize, ansOptions, stream, 0, NoComputeNoVerify);
      compressionManagers.put("ANS", ansManager);

      // Bitcomp
      bitcompOptions = new nvcompBatchedBitcompFormatOpts();
      bitcompOptions.algorithm_type(0); // 0 = default algorithm (best compression), 1 = sparse algorithm (for data with lots of zeros)
      bitcompOptions.data_type(DATA_TYPE);
      bitcompManager = new BitcompManager(chunkSize, bitcompOptions, stream, 0, NoComputeNoVerify);
      compressionManagers.put("Bitcomp", bitcompManager);
   }

   private void runDemo()
   {
      // Read an image from a file
      Mat image = opencv_imgcodecs.imread(FILE_PATH);
      long imageSize = image.elemSize() * image.total();

      Stopwatch stopwatch = new Stopwatch();
      for (Entry<String, PimplManager> managerEntry : compressionManagers.entrySet())
      {
         stopwatch.start();
         BytePointer compressedImageData = new BytePointer();
         long compressedSize = compress(image, compressedImageData, managerEntry.getValue());
         double compressionTime = stopwatch.lap();
         Mat decompressedImage = new Mat(image.size(), image.type());
         decompressImage(compressedImageData, compressedSize, decompressedImage);
         double decompressionTime = stopwatch.lap();

         System.out.println(managerEntry.getKey() + ": ");
         System.out.println("\tCompression Ratio:  " + (double) imageSize / compressedSize);
         System.out.println("\tCompression Time:   " + compressionTime);
         System.out.println("\tDecompression Time: " + decompressionTime);

         checkError(cudaStreamSynchronize(stream));
         checkError(cudaFreeHost(compressedImageData));
         decompressedImage.close();
      }
   }

   private long compress(Mat image, BytePointer compressedImageData, PimplManager compressionManager)
   {
      long decompressedBufferSize = image.elemSize() * image.total();
      BytePointer decompressedDeviceBuffer = allocateToDevice(image);

      BytePointer compressedDeviceBuffer = new BytePointer();
      CompressionConfig compressionConfig = compressionManager.configure_compression(decompressedBufferSize);
      checkError(cudaMallocAsync(compressedDeviceBuffer, compressionConfig.max_compressed_buffer_size(), stream));

      compressionManager.compress(decompressedDeviceBuffer, compressedDeviceBuffer, compressionConfig);
      long compressedBufferSize = compressionManager.get_compressed_output_size(compressedDeviceBuffer);

      checkError(cudaStreamSynchronize(stream));
      cudaMallocHost(compressedImageData, compressedBufferSize);
      checkError(cudaMemcpyAsync(compressedImageData, compressedDeviceBuffer, compressedBufferSize, cudaMemcpyDeviceToHost, stream));

      checkError(cudaFreeAsync(compressedDeviceBuffer, stream));
      checkError(cudaFreeAsync(decompressedDeviceBuffer, stream));
      decompressedDeviceBuffer.close();
      compressedDeviceBuffer.close();
      compressionConfig.close();

      return compressedBufferSize;
   }

   private void decompressImage(BytePointer compressedHostBuffer, long compressedBufferSize, Mat matToPack)
   {
      BytePointer compressedDeviceBuffer = new BytePointer();
      checkError(cudaMallocAsync(compressedDeviceBuffer, compressedBufferSize, stream));
      checkError(cudaMemcpyAsync(compressedDeviceBuffer, compressedHostBuffer, compressedBufferSize, cudaMemcpyDefault, stream));

      nvcompManagerBase decompressionManager = create_manager(compressedDeviceBuffer, stream, 0, NoComputeNoVerify);
      DecompressionConfig decompressionConfig = decompressionManager.configure_decompression(compressedDeviceBuffer);

      BytePointer decompressedDeviceBuffer = new BytePointer();
      checkError(cudaMallocAsync(decompressedDeviceBuffer, decompressionConfig.decomp_data_size(), stream));

      decompressionManager.decompress(decompressedDeviceBuffer, compressedDeviceBuffer, decompressionConfig);

      BytePointer decompressedHostBuffer = new BytePointer();
      checkError(cudaStreamSynchronize(stream));
      checkError(cudaMallocHost(decompressedHostBuffer, decompressionConfig.decomp_data_size()));
      checkError(cudaMemcpyAsync(decompressedHostBuffer, decompressedDeviceBuffer, decompressionConfig.decomp_data_size(), cudaMemcpyDeviceToHost, stream));

      matToPack.data(decompressedHostBuffer);

      checkError(cudaFreeAsync(compressedDeviceBuffer, stream));
      checkError(cudaFreeAsync(decompressedDeviceBuffer, stream));

      compressedDeviceBuffer.close();
      decompressedDeviceBuffer.close();
      decompressionConfig.close();
      decompressionManager.close();
   }

   private BytePointer allocateToDevice(Mat image)
   {
      long imageDataSize = image.elemSize() * image.total();

      BytePointer deviceMemoryPointer = new BytePointer();
      checkError(cudaMallocAsync(deviceMemoryPointer, imageDataSize, stream));
      checkError(cudaMemcpyAsync(deviceMemoryPointer, image.data(), imageDataSize, cudaMemcpyDefault, stream));

      return deviceMemoryPointer;
   }

   private void destroy()
   {
      checkError(cudaStreamSynchronize(stream));
      checkError(cudaStreamDestroy(stream));

      lz4Manager.close();
      lz4Options.close();

      snappyOptions.close();
      snappyManager.close();

      deflateOptions.close();
      deflateManager.close();

      gDeflateOptions.close();
      gDeflateManager.close();

      zstdOptions.close();
      zstdManager.close();

      ansOptions.close();
      ansManager.close();

      bitcompOptions.close();
      bitcompManager.close();
   }

   private void checkError(int errorCode)
   {
      if (errorCode != CUDA_SUCCESS)
      {
         try (BytePointer errorName = cudaGetErrorName(errorCode);
              BytePointer errorString = cudaGetErrorString(errorCode))
         {
            LogTools.error("CUDA Error ({}): {}", errorName.getString(), errorString.getString());
         }
      }
   }

   public static void main(String[] args)
   {
      Loader.load(nvcomp.class);

      new NVCompDemo().runDemo();
   }
}
