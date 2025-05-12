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

   private final int[] sizes = new int[] {0, 0};
   private final SideDependentList<RGBL> sums = new SideDependentList<>(new RGBL(), new RGBL());
   private final SideDependentList<RGBL> sumSquares = new SideDependentList<>(new RGBL(), new RGBL());
   private final SideDependentList<RGB> means = new SideDependentList<>(new RGB(), new RGB());
   private final SideDependentList<RGB> stds = new SideDependentList<>(new RGB(), new RGB());


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
   }

}
