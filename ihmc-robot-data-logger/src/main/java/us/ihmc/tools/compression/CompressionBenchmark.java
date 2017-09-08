package us.ihmc.tools.compression;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.function.DoubleSupplier;

import net.jpountz.lz4.LZ4Compressor;
import net.jpountz.lz4.LZ4Factory;
import us.ihmc.commons.Conversions;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotDataLogger.logger.YoVariableLogReader;

/**
 * Test class to compare the speed and compression factor of various algorithms
 * 
 * @author Jesper Smith
 *
 */
public class CompressionBenchmark extends YoVariableLogReader
{
   //   public static final File directory = new File("/home/jesper/robotLogs/DW17/FinalsTabSequence1");
   public static final File directory = new File("/home/jesper/robotLogs/20170315_165154_AtlasControllerFactory");

   private final ByteBuffer[] trainingSet;
   private final ByteBuffer[] testSet;

   public CompressionBenchmark() throws IOException
   {
      super(directory, new LogPropertiesReader(new File(directory, File.separator + "robotData.log")));

      initialize();

      System.out.println("Loading " + getNumberOfEntries() + " data lines with " + getNumberOfVariables() + " variables.");
      System.out.println("Total memory required " + (((long) getNumberOfEntries() * (long) getNumberOfVariables() * 8) / (1024 * 1024)) + "Mb");

      int trainingSetSize = getNumberOfEntries() / 10;
      int testSetSize = getNumberOfEntries() - trainingSetSize;

      this.trainingSet = new ByteBuffer[trainingSetSize];
      this.testSet = new ByteBuffer[testSetSize];

      for (int i = 0; i < trainingSetSize; i++)
      {
         trainingSet[i] = copyData(i);
      }
      for (int i = 0; i < testSetSize; i++)
      {
         testSet[i] = copyData(i + trainingSetSize);
      }

      System.out.println();
      System.out.println("--- TRAINING (JIT Warmup)---");
      System.out.println();
      benchmark(trainingSet);
      System.out.println();
      System.out.println("--- TEST ---");
      System.out.println();
      benchmark(testSet);
   }

   private void benchmarkFunction(String name, int elements, DoubleSupplier f)
   {
      System.out.println("--- " + name + " ---");
      long start = System.nanoTime();
      double compressionFactor = f.getAsDouble();
      long duration = System.nanoTime() - start;
      System.out.println("Compression factor: " + compressionFactor);
      System.out.println("Duration: " + Conversions.nanosecondsToSeconds(duration) + "s");
      System.out.println("Time per data line: " + Conversions.nanosecondsToMilliseconds((double) (duration / elements)) + "ms");
      System.out.println();
   }

   private void benchmark(ByteBuffer[] set) throws IOException
   {
      benchmarkFunction("Copy", set.length, () -> {
         return benchMarkCopy(set);
      });
      benchmarkFunction("Snappy (Xerial)", set.length, () -> {
         return benchMarkSnappy(set);
      });

      LZ4Compressor safeCompressor = LZ4Factory.safeInstance().fastCompressor();
      benchmarkFunction("LZ4 (Safe)", set.length, () -> {
         return benchMarkLZ4(safeCompressor, set);
      });

      LZ4Compressor unsafeCompressor = LZ4Factory.unsafeInstance().fastCompressor();
      benchmarkFunction("LZ4 (Unsafe)", set.length, () -> {
         return benchMarkLZ4(unsafeCompressor, set);
      });
      
      LZ4Compressor jniCompressor = LZ4Factory.nativeInstance().fastCompressor();
      benchmarkFunction("LZ4 (JNI - java buffer)", set.length, () -> {
         return benchMarkLZ4(jniCompressor, set);
      });

      ByteBuffer directSet[] = new ByteBuffer[set.length];
      for (int i = 0; i < set.length; i++)
      {
         directSet[i] = ByteBuffer.allocateDirect(set[i].capacity());
         set[i].clear();
         directSet[i].put(set[i]);
      }

      benchmarkFunction("LZ4 (Unsafe - direct buffer)", set.length, () -> {
         return benchMarkLZ4(unsafeCompressor, directSet);
      });
      
      benchmarkFunction("LZ4 (JNI - direct buffer)", directSet.length, () -> {
         return benchMarkLZ4(jniCompressor, directSet);
      });
      
      
      benchmarkFunction("Copy (Direct)", directSet.length, () -> {
         return benchMarkCopy(directSet);
      });

   }

   private double benchMarkCopy(ByteBuffer[] set)
   {
      ByteBuffer target;
      if (set[0].isDirect())
      {
         target = ByteBuffer.allocateDirect(getNumberOfVariables() * 8);
      }
      else
      {
         target = ByteBuffer.allocate(getNumberOfVariables() * 8);
      }
      long totalSize = (long) set.length * (long) (getNumberOfVariables() * 8);
      long compressedSize = 0;

      for (int i = 0; i < set.length; i++)
      {
         set[i].clear();
         target.clear();
         target.put(set[i]);
         compressedSize += target.position();
      }
      return ((double) compressedSize) / ((double) totalSize);
   }

   private double benchMarkSnappy(ByteBuffer[] set)
   {
      ByteBuffer target = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(getNumberOfVariables() * 8));

      long totalSize = (long) set.length * (long) (getNumberOfVariables() * 8);
      long compressedSize = 0;

      try
      {
         for (int i = 0; i < set.length; i++)
         {
            set[i].clear();
            target.clear();
            SnappyUtils.compress(set[i], target);
            compressedSize += target.position();
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      return ((double) compressedSize) / ((double) totalSize);
   }

   private double benchMarkLZ4(LZ4Compressor compressor, ByteBuffer[] set)
   {
      ByteBuffer target;
//      if (set[0].isDirect())
      {
         target = ByteBuffer.allocateDirect(compressor.maxCompressedLength(getNumberOfVariables() * 8));
      }
//      else
//      {
//         target = ByteBuffer.allocate(compressor.maxCompressedLength(getNumberOfVariables() * 8));
//      }

      long totalSize = (long) set.length * (long) (getNumberOfVariables() * 8);
      long compressedSize = 0;

      for (int i = 0; i < set.length; i++)
      {
         set[i].clear();
         target.clear();
         compressor.compress(set[i], target);
         compressedSize += target.position();
      }

      return ((double) compressedSize) / ((double) totalSize);
   }

   private ByteBuffer copyData(int i) throws IOException
   {
      ByteBuffer dataLine = readData(i);
      ByteBuffer copy = ByteBuffer.allocate(dataLine.remaining());
      copy.put(dataLine);
      copy.clear();
      return copy;
   }

   public static void main(String[] args) throws IOException
   {
      new CompressionBenchmark();
   }
}
