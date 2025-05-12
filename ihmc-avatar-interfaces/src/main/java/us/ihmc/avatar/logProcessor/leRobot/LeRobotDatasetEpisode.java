package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.yoVariables.variable.YoLong;

import java.nio.file.Path;

public class LeRobotDatasetEpisode
{
   private final int episodeIndex;
   private final String episodeName;
   private final String taskName;
   private final long datasetLengthSoFar;
   private final Path episodesJsonlPath;
   private final Path episodeStatsJsonlPath;
   private final Path dataChunk0Path;
   private final SideDependentList<Path> zedVideoDirs;

   private long length = 0L;

   public LeRobotDatasetEpisode(int episodeIndex,
                                String taskName,
                                long length,
                                long datasetLengthSoFar,
                                Path episodesJsonlPath,
                                Path episodeStatsJsonlPath,
                                Path dataChunk0Path,
                                SideDependentList<Path> zedVideoDirs)
   {
      this.episodeIndex = episodeIndex;
      this.taskName = taskName;
      this.length = length;
      this.datasetLengthSoFar = datasetLengthSoFar;
      this.episodesJsonlPath = episodesJsonlPath;
      this.episodeStatsJsonlPath = episodeStatsJsonlPath;
      this.dataChunk0Path = dataChunk0Path;
      this.zedVideoDirs = zedVideoDirs;

      episodeName = "episode_%06d".formatted(episodeIndex);
   }

   public void startGeneratingEpisode(SCS2LogSessionWithVideo session, Runnable writeMetadataToFilesystem)
   {
      YoLong yoTimestamp = session.getLogDataReader().getTimestamp();

      int inPoint = session.getBufferProperties().getInPoint();
      int outPoint = session.getBufferProperties().getOutPoint();
      double sessionDTSeconds = session.getSessionDTSeconds();
      LogTools.info("dt: {}", sessionDTSeconds);

      ThreadTools.startAsDaemon(() ->
      {
         session.submitBufferIndexRequestAndWait(inPoint);

         ZEDSVOScrubber zedSVOScrubber = session.getZedSVOScrubbers().get(0);

         SideDependentList<LeRobotDatasetVideoWriter> ffmpegRecorders = new SideDependentList<>();
         for (RobotSide side : RobotSide.values)
         {
            ffmpegRecorders.put(side, new LeRobotDatasetVideoWriter(side, zedVideoDirs.get(side).resolve(episodeName + ".mp4")));
         }

         LeRobotDatasetDataWriter dataWriter = new LeRobotDatasetDataWriter(episodeIndex, datasetLengthSoFar, session.getRootRegistry());

         length = 0L;
         long startVideoTimestamp = -1;
         long lastVideoTimestamp = -1;
         for (int i = 0; i < outPoint - inPoint; i++)
         {
            session.playbackTick(); // TODO: Is this skipping the first tick?

            long timestamp = yoTimestamp.getLongValue();
            zedSVOScrubber.scrub(timestamp);
            long currentVideoTimestamp = zedSVOScrubber.getTimestampScrubber().getCurrentVideoTimestamp();
            if (startVideoTimestamp < 0)
               startVideoTimestamp = currentVideoTimestamp;

            if (currentVideoTimestamp > lastVideoTimestamp) // Write only when a new frame is available
            {
               double period = 0.0;
               if (lastVideoTimestamp >= 0)
                  period = Conversions.nanosecondsToSeconds(currentVideoTimestamp - lastVideoTimestamp);

               lastVideoTimestamp = currentVideoTimestamp;

               long videoTimestampMicros = Math.round((currentVideoTimestamp - startVideoTimestamp) / 1000.0);
               LogTools.info("Current timestamp: %d  Writing frame %.3f Frequency %.3f".formatted(currentVideoTimestamp,
                                                                                                  videoTimestampMicros / 1000.0,
                                                                                                  Conversions.secondsToHertz(period)));
               for (RobotSide side : RobotSide.values)
               {
                  ffmpegRecorders.get(side).writeFrame(zedSVOScrubber, videoTimestampMicros);
               }

               dataWriter.addFrame(videoTimestampMicros, length);

               ++length;
            }
         }

         LogTools.info("Wrote episode with %d frames.".formatted(length));
         for (RobotSide side : RobotSide.values)
         {
            ffmpegRecorders.get(side).close();
         }

         Path parquetPath = dataChunk0Path.resolve(episodeName + ".parquet");
         dataWriter.writeFile(parquetPath);

         writeMetadataToFilesystem.run();

         appendJsonStatsLine();

      }, getClass().getSimpleName());
   }

   public void appendJsonBasicEntryLine()
   {
      LeRobotDatasetTools.appendLine(episodesJsonlPath, JSONFileTools.getAsSingleLine(node ->
      {
         node.put("episode_index", episodeIndex);
         ArrayNode tasksArray = node.putArray("tasks");
         tasksArray.add(taskName);
         node.put("length", length);
      }));
   }

   public void appendJsonStatsLine()
   {

      // TODO

      LeRobotDatasetTools.appendLine(episodeStatsJsonlPath, JSONFileTools.getAsSingleLine(node ->
      {
         node.put("episode_index", episodeIndex);
         ObjectNode stats = node.putObject("stats");
         for (RobotSide side : RobotSide.values)
         {
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
            ArrayNode mean = video.putArray("mean");
            mean.addArray().addArray().add(0.5f);
            mean.addArray().addArray().add(0.5f);
            mean.addArray().addArray().add(0.5f);
            ArrayNode std = video.putArray("std");
            std.addArray().addArray().add(0.3f);
            std.addArray().addArray().add(0.3f);
            std.addArray().addArray().add(0.3f);
            video.putArray("count").add(length);
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
         fieldStats.putArray("min").add(0);
         fieldStats.putArray("max").add(0);
         fieldStats.putArray("mean").add(0.0f);
         fieldStats.putArray("std").add(0.0f);
         fieldStats.putArray("count").add(length);
         fieldStats = stats.putObject("frame_index");
         fieldStats.putArray("min").add(0);
         fieldStats.putArray("max").add(length - 1);
         fieldStats.putArray("mean").add((length - 1) / 2.0f);
         fieldStats.putArray("std").add(0.0f);
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
         fieldStats.putArray("min").add(0);
         fieldStats.putArray("max").add(length - 1);
         fieldStats.putArray("mean").add((length - 1) / 2.0f);
         fieldStats.putArray("std").add(0.0f);
         fieldStats.putArray("count").add(length);
         fieldStats = stats.putObject("task_index");
         fieldStats.putArray("min").add(0);
         fieldStats.putArray("max").add(0);
         fieldStats.putArray("mean").add(0.0f);
         fieldStats.putArray("std").add(0.0f);
         fieldStats.putArray("count").add(length);
      }));
   }

   public String getEpisodeName()
   {
      return episodeName;
   }

   public Path getEpisodesJsonlPath()
   {
      return episodesJsonlPath;
   }

   public Path getEpisodeStatsJsonlPath()
   {
      return episodeStatsJsonlPath;
   }

   public Path getDataChunk0Path()
   {
      return dataChunk0Path;
   }

   public SideDependentList<Path> getZedVideoDirs()
   {
      return zedVideoDirs;
   }

   public int getLength()
   {
      return (int) length;
   }
}
