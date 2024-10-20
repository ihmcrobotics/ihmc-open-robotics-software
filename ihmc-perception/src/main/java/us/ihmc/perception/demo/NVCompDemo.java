package us.ihmc.perception.demo;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.global.nvcomp;
import org.bytedeco.cuda.nvcomp.ANSManager;
import org.bytedeco.cuda.nvcomp.BitcompManager;
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
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.perception.cuda.CUDACompressionTools;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.Map;
import java.util.Map.Entry;
import java.util.TreeMap;

import static org.bytedeco.cuda.global.cudart.cudaFreeHost;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;
import static org.bytedeco.cuda.global.nvcomp.*;
import static us.ihmc.perception.cuda.CUDATools.checkCUDAError;

public class NVCompDemo
{
   private static final WorkspaceResourceDirectory RESOURCE_DIRECTORY = new WorkspaceResourceDirectory(NVCompDemo.class);
   private static final WorkspaceResourceFile IMAGE_FILE = new WorkspaceResourceFile(RESOURCE_DIRECTORY, "ZED_ColorImage.png");

   protected static final int DATA_TYPE = NVCOMP_TYPE_CHAR;

   protected final CUstream_st stream = CUDAStreamManager.getStream();
   protected final long chunkSize = 1 << 16;

   // LZ4
   protected final LZ4Manager lz4Manager;
   protected final nvcompBatchedLZ4Opts_t lz4Options;

   // Snappy
   protected final SnappyManager snappyManager;
   protected final nvcompBatchedSnappyOpts_t snappyOptions;

   // Deflate
   protected final DeflateManager deflateManager;
   protected final nvcompBatchedDeflateOpts_t deflateOptions;

   // GDeflate
   protected final GdeflateManager gDeflateManager;
   protected final nvcompBatchedGdeflateOpts_t gDeflateOptions;

   // ZSTD
   protected final ZstdManager zstdManager;
   protected final nvcompBatchedZstdOpts_t zstdOptions;

   // ANS
   protected final ANSManager ansManager;
   protected final nvcompBatchedANSOpts_t ansOptions;

   // Bitcomp
   protected final BitcompManager bitcompManager;
   protected final nvcompBatchedBitcompFormatOpts bitcompOptions;

   protected final Map<String, PimplManager> compressionManagers = new TreeMap<>();

   protected NVCompDemo()
   {
      LogTools.info("Chunk Size: " + chunkSize);

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
      deflateOptions = new nvcompBatchedDeflateOpts_t();
      deflateManager = new DeflateManager(chunkSize, deflateOptions, stream, 0, NoComputeNoVerify);
      compressionManagers.put("Deflate", deflateManager);

      // GDeflate
      gDeflateOptions = new nvcompBatchedGdeflateOpts_t();
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

   protected void runDemo()
   {
      // Read an image from a file
      Mat image = opencv_imgcodecs.imread(IMAGE_FILE.getClasspathResource().getFile());
      long imageSize = image.elemSize() * image.total();

      Stopwatch stopwatch = new Stopwatch();
      for (Entry<String, PimplManager> managerEntry : compressionManagers.entrySet())
      {
         stopwatch.start();
         BytePointer compressedImageData = CUDACompressionTools.compress(image.data(), imageSize, managerEntry.getValue(), stream);
         long compressedSize = compressedImageData.limit();
         double compressionTime = stopwatch.lap();
         BytePointer decompressedImageData = CUDACompressionTools.decompress(compressedImageData, compressedSize, false, stream);
         Mat decompressedImage = new Mat(image.size(), image.type(), decompressedImageData);
         double decompressionTime = stopwatch.lap();

         System.out.println(managerEntry.getKey() + ": ");
         System.out.println("\tCompression Ratio:  " + (double) imageSize / compressedSize);
         System.out.println("\tCompression Time:   " + compressionTime);
         System.out.println("\tDecompression Time: " + decompressionTime);

         checkCUDAError(cudaStreamSynchronize(stream));
         checkCUDAError(cudaFreeHost(compressedImageData));
         decompressedImage.close();
      }
   }

   protected void destroy()
   {
      checkCUDAError(cudaStreamSynchronize(stream));
      CUDAStreamManager.releaseStream(stream);

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

   public static void main(String[] args)
   {
      Loader.load(nvcomp.class);

      NVCompDemo demo = new NVCompDemo();
      demo.runDemo();
      demo.destroy();
   }
}
