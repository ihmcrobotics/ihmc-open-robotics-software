package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;

/**
 * Calculates data statistics, which are used to normalize the machine learning data
 * before training.
 * There is no reset function. Just create a whole new one to do it again.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 */
public class LeRobotDatasetEpisodeStatistics
{
   private class RGBL
   {
      long r, g, b = 0L;
   }
   private class RGB
   {
      float r, g, b = 0.0f;
   }

   // image stats
   private final int[] sizes = new int[] {0, 0};
   private final SideDependentList<RGBL> sums = new SideDependentList<>(new RGBL(), new RGBL());
   private final SideDependentList<RGBL> sumSquares = new SideDependentList<>(new RGBL(), new RGBL());
   private final SideDependentList<RGB> means = new SideDependentList<>(new RGB(), new RGB());
   private final SideDependentList<RGB> stds = new SideDependentList<>(new RGB(), new RGB());

   // data stats
   private int length = 0;
   private final List<LeRobotFloatStatisticsCalculator> stateStats = new java.util.ArrayList<>();
   private final List<LeRobotFloatStatisticsCalculator> actionStats = new java.util.ArrayList<>();
   private final LeRobotIntegerStatisticsCalculator episodeIndexStats = new LeRobotIntegerStatisticsCalculator();
   private final LeRobotIntegerStatisticsCalculator frameIndexStats = new LeRobotIntegerStatisticsCalculator();
   private final LeRobotFloatStatisticsCalculator timestampStats = new LeRobotFloatStatisticsCalculator();
   private final LeRobotIntegerStatisticsCalculator nextDoneStats = new LeRobotIntegerStatisticsCalculator();
   private final LeRobotIntegerStatisticsCalculator indexStats = new LeRobotIntegerStatisticsCalculator();
   private final LeRobotIntegerStatisticsCalculator taskIndexStats = new LeRobotIntegerStatisticsCalculator();

   public void submitFrame(RobotSide side, Mat bgrMat)
   {
      // Calculate average pixel values in RGB channels, normalized to 0.0-1.0
      // Get image dimensions
      int height = bgrMat.rows();
      int width = bgrMat.cols();
      int channels = bgrMat.channels();
      int totalPixels = height * width;
      sizes[side.ordinal()] += totalPixels;

      RGBL sum = sums.get(side);
      RGBL sumSq = sumSquares.get(side);
   
      // Get raw image data
      byte[] data = new byte[width * height * channels];
      bgrMat.data().get(data);
   
      // Sum up all RGB values and their squares (for stddev calculation)
      for (int i = 0; i < totalPixels; i++)
      {
         int offset = i * channels;
         // Assuming RGBA or RGB format (channels are either 3 or 4)
         // OpenCV uses BGR ordering, so we need to map accordingly
         int b = data[offset] & 0xFF;      // B is first in OpenCV
         int g = data[offset + 1] & 0xFF;  // G is second
         int r = data[offset + 2] & 0xFF;  // R is third
         
         sum.b += b;
         sum.g += g;
         sum.r += r;
         
         sumSq.b += (long) b * b;
         sumSq.g += (long) g * g;
         sumSq.r += (long) r * r;
      }
   }

   public void processParquetRecord(LeRobotEpisodeRecord dataFrame)
   {
      for (int i = 0; i < dataFrame.state().size(); i++)
      {
         if (length == 0)
            stateStats.add(new LeRobotFloatStatisticsCalculator());

         stateStats.get(i).addValue(dataFrame.state().get(i));
      }

      for (int i = 0; i < dataFrame.action().size(); i++)
      {
         if (length == 0)
            actionStats.add(new LeRobotFloatStatisticsCalculator());

         actionStats.get(i).addValue(dataFrame.action().get(i));
      }

      episodeIndexStats.addValue(dataFrame.episodeIndex());
      frameIndexStats.addValue(dataFrame.frameIndex());
      timestampStats.addValue(dataFrame.timestamp());
      nextDoneStats.addValue(dataFrame.nextDone() ? 1 : 0);
      indexStats.addValue(dataFrame.index());
      taskIndexStats.addValue(dataFrame.taskIndex());

      ++length;
   }

   public void calculate()
   {
      for (RobotSide side : RobotSide.values)
      {
         int totalPixels = sizes[side.ordinal()];
         RGBL sum = sums.get(side);
         RGBL sumSq = sumSquares.get(side);
         RGB mean = means.get(side);
         RGB std = stds.get(side);
         if (totalPixels == 0)
            continue;
         
         // Calculate averages and normalize to 0.0-1.0
         mean.r = (float) sum.r / (totalPixels * 255.0f);
         mean.g = (float) sum.g / (totalPixels * 255.0f);
         mean.b = (float) sum.b / (totalPixels * 255.0f);
         
         // Calculate standard deviations and normalize to 0.0-1.0
         // stddev = sqrt((sum_of_squares/n) - (mean)²)
         double meanR = (double) sum.r / totalPixels;
         double meanG = (double) sum.g / totalPixels;
         double meanB = (double) sum.b / totalPixels;
         
         double varR = ((double) sumSq.r / totalPixels) - (meanR * meanR);
         double varG = ((double) sumSq.g / totalPixels) - (meanG * meanG);
         double varB = ((double) sumSq.b / totalPixels) - (meanB * meanB);
         
         std.r = (float) (Math.sqrt(Math.max(0, varR)) / 255.0f);
         std.g = (float) (Math.sqrt(Math.max(0, varG)) / 255.0f);
         std.b = (float) (Math.sqrt(Math.max(0, varB)) / 255.0f);
   
         LogTools.info("Mean RGB: R=%.3f G=%.3f B=%.3f".formatted(mean.r, mean.g, mean.b));
         LogTools.info("StdDev RGB: R=%.3f G=%.3f B=%.3f".formatted(std.r, std.g, std.b));
      }

      for (LeRobotFloatStatisticsCalculator calculator : stateStats)
         calculator.calculate();
      for (LeRobotFloatStatisticsCalculator calculator : actionStats)
         calculator.calculate();
      episodeIndexStats.calculate();
      frameIndexStats.calculate();
      timestampStats.calculate();
      nextDoneStats.calculate();
      indexStats.calculate();
      taskIndexStats.calculate();
   }

   /**
    * @param videoFeatureKeys  feature name for each side, e.g. "observation.images.cam_zed_left"
    */
   public void writeJson(ObjectNode stats, SideDependentList<String> videoFeatureKeys)
   {
      for (RobotSide side : RobotSide.values)
      {
         String featureKey = videoFeatureKeys.get(side);
         if (featureKey == null)
            continue;

         // Always write the camera key so lerobot's dataset.meta.stats lookup never KeyErrors.
         // If no frames were processed the pixel stats default to 0 (safe since pi0_fast uses VISUAL: IDENTITY).
         RGB mean = means.get(side);
         RGB std = stds.get(side);

         // RGB
         ObjectNode video = stats.putObject(featureKey);
         addRGB(video.putArray("min"), 0.0f, 0.0f, 0.0f); // Looks like we can leave 0.0
         addRGB(video.putArray("max"), 1.0f, 1.0f, 1.0f); // Looks like we can leave 1.0
         addRGB(video.putArray("mean"), mean.r, mean.g, mean.b);
         addRGB(video.putArray("std"), std.r, std.g, std.b);
         video.putArray("count").add(sizes[side.ordinal()]);
      }

      writeVectorStats(stats, "observation.state", stateStats);
      writeVectorStats(stats, "action", actionStats);
      writeScalarStats(stats, "episode_index", episodeIndexStats, false);
      writeScalarStats(stats, "frame_index", frameIndexStats, false);
      writeScalarStats(stats, "timestamp", timestampStats);
      writeScalarStats(stats, "next.done", nextDoneStats, true);
      writeScalarStats(stats, "index", indexStats, false);
      writeScalarStats(stats, "task_index", taskIndexStats, false);
   }

   private static void addRGB(ArrayNode node, float r, float g, float b)
   {
      node.addArray().addArray().add(r);
      node.addArray().addArray().add(g);
      node.addArray().addArray().add(b);
   }

   private void writeVectorStats(ObjectNode stats, String name, List<LeRobotFloatStatisticsCalculator> calculators)
   {
      ObjectNode fieldStats = stats.putObject(name);
      ArrayNode min = fieldStats.putArray("min");
      ArrayNode max = fieldStats.putArray("max");
      ArrayNode mean = fieldStats.putArray("mean");
      ArrayNode std = fieldStats.putArray("std");
      for (LeRobotFloatStatisticsCalculator calculator : calculators)
      {
         min.add(calculator.getMin());
         max.add(calculator.getMax());
         mean.add(calculator.getMean());
         std.add(calculator.getStddev());
      }
      fieldStats.putArray("count").add(length);
   }

   private void writeScalarStats(ObjectNode stats, String name, LeRobotFloatStatisticsCalculator calculator)
   {
      ObjectNode fieldStats = stats.putObject(name);
      fieldStats.putArray("min").add(calculator.getMin());
      fieldStats.putArray("max").add(calculator.getMax());
      fieldStats.putArray("mean").add(calculator.getMean());
      fieldStats.putArray("std").add(calculator.getStddev());
      fieldStats.putArray("count").add(length);
   }

   private void writeScalarStats(ObjectNode stats, String name, LeRobotIntegerStatisticsCalculator calculator, boolean booleanExtrema)
   {
      ObjectNode fieldStats = stats.putObject(name);
      if (booleanExtrema)
      {
         fieldStats.putArray("min").add(calculator.getMin() == 1);
         fieldStats.putArray("max").add(calculator.getMax() == 1);
      }
      else
      {
         fieldStats.putArray("min").add(calculator.getMin());
         fieldStats.putArray("max").add(calculator.getMax());
      }
      fieldStats.putArray("mean").add(calculator.getMean());
      fieldStats.putArray("std").add(calculator.getStddev());
      fieldStats.putArray("count").add(length);
   }

   public void mergeFrom(LeRobotDatasetEpisodeStatistics other)
   {
      length += other.length;
      for (RobotSide side : RobotSide.values)
      {
         sizes[side.ordinal()] += other.sizes[side.ordinal()];
         RGBL mySum = sums.get(side);
         RGBL otherSum = other.sums.get(side);
         mySum.r += otherSum.r;
         mySum.g += otherSum.g;
         mySum.b += otherSum.b;
         RGBL mySumSq = sumSquares.get(side);
         RGBL otherSumSq = other.sumSquares.get(side);
         mySumSq.r += otherSumSq.r;
         mySumSq.g += otherSumSq.g;
         mySumSq.b += otherSumSq.b;
      }
      if (stateStats.isEmpty() && !other.stateStats.isEmpty())
         for (int i = 0; i < other.stateStats.size(); i++)
            stateStats.add(new LeRobotFloatStatisticsCalculator());
      for (int i = 0; i < Math.min(stateStats.size(), other.stateStats.size()); i++)
         stateStats.get(i).mergeFrom(other.stateStats.get(i));
      if (actionStats.isEmpty() && !other.actionStats.isEmpty())
         for (int i = 0; i < other.actionStats.size(); i++)
            actionStats.add(new LeRobotFloatStatisticsCalculator());
      for (int i = 0; i < Math.min(actionStats.size(), other.actionStats.size()); i++)
         actionStats.get(i).mergeFrom(other.actionStats.get(i));
      episodeIndexStats.mergeFrom(other.episodeIndexStats);
      frameIndexStats.mergeFrom(other.frameIndexStats);
      timestampStats.mergeFrom(other.timestampStats);
      nextDoneStats.mergeFrom(other.nextDoneStats);
      indexStats.mergeFrom(other.indexStats);
      taskIndexStats.mergeFrom(other.taskIndexStats);
   }

   public void loadJSON(JsonNode lineRoot)
   {
      // TODO
   }
}
