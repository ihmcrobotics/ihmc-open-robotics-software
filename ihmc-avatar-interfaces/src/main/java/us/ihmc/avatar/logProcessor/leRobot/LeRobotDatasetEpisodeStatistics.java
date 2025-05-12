package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.nio.file.Path;

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
   private final LeRobotIntegerStatisticsCalculator episodeIndexStats = new LeRobotIntegerStatisticsCalculator();
   private final LeRobotIntegerStatisticsCalculator frameIndexStats = new LeRobotIntegerStatisticsCalculator();
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
      ++length;

      // Track statistics for integer fields using the statistics calculator
      episodeIndexStats.addValue(dataFrame.episodeIndex());
      frameIndexStats.addValue(dataFrame.frameIndex());
      indexStats.addValue(dataFrame.index());
      taskIndexStats.addValue(dataFrame.taskIndex());
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

      episodeIndexStats.calculate();
      frameIndexStats.calculate();
      indexStats.calculate();
      taskIndexStats.calculate();
   }

   public void writeJson(ObjectNode stats, SideDependentList<Path> zedVideoDirs)
   {
      for (RobotSide side : RobotSide.values)
      {
         RGB mean = means.get(side);
         RGB std = stds.get(side);
   
         // RGB
         ObjectNode video = stats.putObject(zedVideoDirs.get(side).getFileName().toString());
         ArrayNode min = video.putArray("min"); // Looks like we can leave 0.0
         min.addArray().addArray().add(0.0f);
         min.addArray().addArray().add(0.0f);
         min.addArray().addArray().add(0.0f);
         ArrayNode max = video.putArray("max"); // Looks like we can leave 1.0
         max.addArray().addArray().add(1.0f);
         max.addArray().addArray().add(1.0f);
         max.addArray().addArray().add(1.0f);
         ArrayNode meanNode = video.putArray("mean");
         meanNode.addArray().addArray().add(mean.r);
         meanNode.addArray().addArray().add(mean.g);
         meanNode.addArray().addArray().add(mean.b);
         ArrayNode stdNode = video.putArray("std");
         stdNode.addArray().addArray().add(std.r);
         stdNode.addArray().addArray().add(std.g);
         stdNode.addArray().addArray().add(std.b);
         video.putArray("count").add(sizes[side.ordinal()]);
      }

      ObjectNode state = stats.putObject("state");
      ArrayNode min = state.putArray("min");
      for (int i = 0; i < 14; i++)
         min.add(0.0f);
      ArrayNode max = state.putArray("max");
      for (int i = 0; i < 14; i++)
         max.add(10.0f);
      ArrayNode mean = state.putArray("mean");
      for (int i = 0; i < 14; i++)
         mean.add(5.0f);
      ArrayNode std = state.putArray("std");
      for (int i = 0; i < 14; i++)
         std.add(3.0f);
      state.putArray("count").add(length);

      ObjectNode action = stats.putObject("action");
      min = action.putArray("min");
      for (int i = 0; i < 14; i++)
         min.add(0.0f);
      max = action.putArray("max");
      for (int i = 0; i < 14; i++)
         max.add(10.0f);
      mean = action.putArray("mean");
      for (int i = 0; i < 14; i++)
         mean.add(5.0f);
      std = action.putArray("std");
      for (int i = 0; i < 14; i++)
         std.add(3.0f);
      action.putArray("count").add(length);

      ObjectNode fieldStats = stats.putObject("episode_index");
      fieldStats.putArray("min").add(episodeIndexStats.getMin());
      fieldStats.putArray("max").add(episodeIndexStats.getMax());
      fieldStats.putArray("mean").add(episodeIndexStats.getMean());
      fieldStats.putArray("std").add(episodeIndexStats.getStddev());
      fieldStats.putArray("count").add(length);
      
      fieldStats = stats.putObject("frame_index");
      fieldStats.putArray("min").add(frameIndexStats.getMin());
      fieldStats.putArray("max").add(frameIndexStats.getMax());
      fieldStats.putArray("mean").add(frameIndexStats.getMean());
      fieldStats.putArray("std").add(frameIndexStats.getStddev());
      fieldStats.putArray("count").add(length);
      
      fieldStats = stats.putObject("timestamp");
      fieldStats.putArray("min").add(0);
      fieldStats.putArray("max").add(0);
      fieldStats.putArray("mean").add(0.0f);
      fieldStats.putArray("std").add(0.0f);
      fieldStats.putArray("count").add(length);
      
      fieldStats = stats.putObject("next.done");
      fieldStats.putArray("min").add(false);
      fieldStats.putArray("max").add(true);
      fieldStats.putArray("mean").add(0.00066f);
      fieldStats.putArray("std").add(0.025f);
      fieldStats.putArray("count").add(length);
      
      fieldStats = stats.putObject("index");
      fieldStats.putArray("min").add(indexStats.getMin());
      fieldStats.putArray("max").add(indexStats.getMax());
      fieldStats.putArray("mean").add(indexStats.getMean());
      fieldStats.putArray("std").add(indexStats.getStddev());
      fieldStats.putArray("count").add(length);
      
      fieldStats = stats.putObject("task_index");
      fieldStats.putArray("min").add(taskIndexStats.getMin());
      fieldStats.putArray("max").add(taskIndexStats.getMax());
      fieldStats.putArray("mean").add(taskIndexStats.getMean());
      fieldStats.putArray("std").add(taskIndexStats.getStddev());
      fieldStats.putArray("count").add(length);
   }
}
