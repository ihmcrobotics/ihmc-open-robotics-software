package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUevent_st;
import us.ihmc.log.LogTools;

import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Optional;

import static org.bytedeco.cuda.global.cudart.*;

public class CUDATimingTools
{
   private static final int MAX_ENTRIES = 100;
   private static final Map<String, Deque<Float>> executionTimes = new HashMap<>();

   private static CUevent_st start;
   private static CUevent_st end;

   public static void startKernelTimer()
   {
      start = new CUevent_st();
      end = new CUevent_st();

      cudaEventCreate(start);
      cudaEventCreate(end);

      cudaEventRecord(start);
   }

   public static void endKernelTimer(String kernelName)
   {
      cudaEventRecord(end);
      cudaEventSynchronize(end);

      float[] milliseconds = new float[1];
      milliseconds[0] = 0.0f;
      cudaEventElapsedTime(milliseconds, start, end);

      executionTimes.computeIfAbsent(kernelName, k -> new LinkedList<>()).add(milliseconds[0]);

      if (executionTimes.get(kernelName).size() > MAX_ENTRIES)
      {
         executionTimes.get(kernelName).pollFirst();
      }

      LogTools.info("Update kernel run time for " + kernelName + " in milliseconds: " + milliseconds[0]);
   }

   public static double getAverageTime(String kernelName)
   {
      Deque<Float> times = executionTimes.get(kernelName);
      if (times == null || times.isEmpty())
      {
         LogTools.info("No recorded times for " + kernelName);
         return Float.NaN;
      }

      return times.stream().mapToDouble(Float::doubleValue).average().orElse(0.0);
   }

   public static Float getMinTime(String kernelName)
   {
      Deque<Float> times = executionTimes.get(kernelName);
      if (times == null || times.isEmpty())
      {
         LogTools.info("No recorded times for " + kernelName);
         return Float.NaN;
      }
      Optional<Float> min = times.stream().min(Float::compareTo);
      return min.orElse(null);
   }

   public static Float getMaxTime(String kernelName)
   {
      Deque<Float> times = executionTimes.get(kernelName);
      if (times == null || times.isEmpty())
      {
         LogTools.info("No recorded times for " + kernelName);
         return Float.NaN;
      }

      Optional<Float> max = times.stream().max(Float::compareTo);
      return max.orElse(null);
   }

   public static double getStandardDeviation(String kernelName)
   {
      Deque<Float> times = executionTimes.get(kernelName);
      if (times == null || times.isEmpty())
      {
         LogTools.info("No recorded times for " + kernelName);
         return Float.NaN;
      }
      double average = times.stream().mapToDouble(Float::doubleValue).average().orElse(0.0);
      double variance = times.stream().mapToDouble(time -> Math.pow(time - average, 2)).average().orElse(0.0);
      return Math.sqrt(variance);
   }

   public static void printTimesForKernel(String kernelName)
   {
      Deque<Float> times = executionTimes.get(kernelName);
      if (times == null || times.isEmpty())
      {
         LogTools.info("No recorded times for " + kernelName);
      }

      double average = getAverageTime(kernelName);
      double variance = getStandardDeviation(kernelName);
      double min = getMinTime(kernelName);
      double max = getMaxTime(kernelName);

      LogTools.info("Timings for kernel " + kernelName + " in milliseconds!");
      LogTools.info("|   Average time: " + average);
      LogTools.info("|   Variance time: " + variance);
      LogTools.info("|   Min time: " + min);
      LogTools.info("|   Max time: " + max);
      LogTools.warn("******************************************");

   }

}
