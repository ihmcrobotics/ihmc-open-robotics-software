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

      // Get raw image data
      byte[] data = new byte[width * height * channels];
      bgrMat.data().get(data);

      // Sum up all RGB values
      for (int i = 0; i < totalPixels; i++)
      {
         int offset = i * channels;
         // Assuming RGBA or RGB format (channels are either 3 or 4)
         // OpenCV uses BGR ordering, so we need to map accordingly
         sum.b += data[offset] & 0xFF;     // B is first in OpenCV
         sum.g += data[offset + 1] & 0xFF; // G is second
         sum.r += data[offset + 2] & 0xFF; // R is third
      }
   }

   public void calculate()
   {
      for (RobotSide side : RobotSide.values)
      {
         int totalPixels = sizes[side.ordinal()];
         RGBL sum = sums.get(side);
         RGB mean = means.get(side);
         // Calculate averages and normalize to 0.0-1.0
         mean.r = (float) sum.r / (totalPixels * 255.0f);
         mean.g = (float) sum.g / (totalPixels * 255.0f);
         mean.b = (float) sum.b / (totalPixels * 255.0f);

         LogTools.info("Mean RGB: R=%.3f G=%.3f B=%.3f".formatted(mean.r, mean.g, mean.b));
      }
   }

   public void writeJson(ObjectNode stats, SideDependentList<Path> zedVideoDirs)
   {
      for (RobotSide side : RobotSide.values)
      {
         RGB mean = means.get(side);

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
         ArrayNode std = video.putArray("std");
         std.addArray().addArray().add(0.3f);
         std.addArray().addArray().add(0.3f);
         std.addArray().addArray().add(0.3f);
         video.putArray("count").add(sizes[side.ordinal()]);
      }
   }

}
