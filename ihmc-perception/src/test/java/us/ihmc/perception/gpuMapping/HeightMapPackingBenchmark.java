package us.ihmc.perception.gpuMapping;

import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Fork;
import org.openjdk.jmh.annotations.Level;
import org.openjdk.jmh.annotations.Measurement;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;
import org.openjdk.jmh.annotations.Warmup;
import org.openjdk.jmh.infra.Blackhole;
import org.openjdk.jmh.runner.Runner;
import org.openjdk.jmh.runner.options.Options;
import org.openjdk.jmh.runner.options.OptionsBuilder;
import perception_msgs.msg.dds.HeightMapMessage;

import java.util.concurrent.TimeUnit;

@State(Scope.Thread)
@BenchmarkMode(Mode.AverageTime)
@OutputTimeUnit(TimeUnit.MICROSECONDS)
@Warmup(iterations = 5, time = 1)
@Measurement(iterations = 10, time = 1)
@Fork(value = 0) // Runs in the host VM, no new process spawned
public class HeightMapPackingBenchmark
{
   private static final double CELL_RESOLUTION = 0.05;
   private static final double WIDTH_IN_METERS = 5.0;

   private HeightMapData heightMapData;
   private HeightMapMessage heightMapMessage;

   @Setup(Level.Trial)
   public void setup()
   {
      heightMapData = new HeightMapData(CELL_RESOLUTION, WIDTH_IN_METERS, 0.0, 0.0);
      heightMapMessage = new HeightMapMessage();

      int centerIndex = HeightMapTools.computeCenterIndex(WIDTH_IN_METERS, CELL_RESOLUTION);
      int cellsPerAxis = 2 * centerIndex + 1;
      int totalCells = cellsPerAxis * cellsPerAxis;

      for (int i = 0; i < totalCells; i++)
      {
         heightMapData.setHeight(i, 1.0f);
      }
   }

   @Benchmark
   public void testToMessage(Blackhole bh)
   {
      // The code under test
      HeightMapMessageTools.toMessage(heightMapData, heightMapMessage);

      // Ensure the JIT doesn't optimize the call away
      bh.consume(heightMapMessage);
   }

   public static void main(String[] args) throws Exception {
      Options opt = new OptionsBuilder()
            .include(HeightMapPackingBenchmark.class.getSimpleName())
            .forks(1)
            .build();

      new Runner(opt).run();
   }
}