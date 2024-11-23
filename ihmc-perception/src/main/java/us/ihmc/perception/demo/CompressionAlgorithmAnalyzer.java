package us.ihmc.perception.demo;

import org.bytedeco.javacpp.Pointer;
import us.ihmc.commons.Conversions;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;

public class CompressionAlgorithmAnalyzer
{
   /**
    * The compression BiFunction takes in pointers to host and device memory
    * pointing to the data that should be compressed and the data size,
    * compresses the data, and returns the size of the compressed data.
    */
   private final BiFunction<Pointer, Pointer, Long> compressionBiFunction;

   private final List<Long> postCompressionSizes = new ArrayList<>();
   private final List<Double> compressionRatios = new ArrayList<>();
   private final List<Double> compressionDurations = new ArrayList<>();

   public CompressionAlgorithmAnalyzer(BiFunction<Pointer, Pointer, Long> compressionBiFunction)
   {
      this.compressionBiFunction = compressionBiFunction;
   }

   public void testCompression(Pointer cpuData, Pointer gpuData, long dataSize)
   {
      long startTime = System.nanoTime();
      long postCompressionSize = compressionBiFunction.apply(cpuData, gpuData);
      long endTime = System.nanoTime();
      postCompressionSizes.add(postCompressionSize);
      compressionRatios.add((double) dataSize / postCompressionSize);
      compressionDurations.add(Conversions.nanosecondsToSeconds(endTime - startTime));
   }

   public long totalCompressedDataSize()
   {
      return postCompressionSizes.stream().reduce(Long::sum).orElse(0L);
   }

   public double totalCompressionDuration()
   {
      return compressionDurations.stream().reduce(Double::sum).orElse(0.0);
   }

   public double averageCompressedDataSize()
   {
      return (double) totalCompressedDataSize() / postCompressionSizes.size();
   }

   public double averageCompressionRatio()
   {
      return compressionRatios.stream().reduce(Double::sum).orElse(0.0) / compressionRatios.size();
   }

   public double averageCompressionDuration()
   {
      return totalCompressionDuration() / compressionDurations.size();
   }
}
