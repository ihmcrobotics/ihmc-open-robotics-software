package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.node.ArrayNode;
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
   private final Path episodesJsonlPath;
   private final Path episodeStatsJsonlPath;
   private final Path dataChunk0Path;
   private final SideDependentList<Path> zedVideoDirs;

   private int length = 0; // TODO
   private final SideDependentList<LeRobotDatasetVideoWriter> ffmpegRecorders = new SideDependentList<>();

   public LeRobotDatasetEpisode(int episodeIndex,
                                String taskName,
                                int length,
                                Path episodesJsonlPath,
                                Path episodeStatsJsonlPath,
                                Path dataChunk0Path,
                                SideDependentList<Path> zedVideoDirs)
   {
      this.episodeIndex = episodeIndex;
      this.taskName = taskName;
      this.length = length;
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
         int imageHeight = zedSVOScrubber.getImageHeight();
         int imageWidth = zedSVOScrubber.getImageWidth();

         for (RobotSide side : RobotSide.values)
         {
            ffmpegRecorders.put(side, new LeRobotDatasetVideoWriter(side, zedVideoDirs.get(side).resolve(episodeName + ".mp4")));
         }

         length = 0;
         long lastVideoTimestamp = -1;
         int scsInOutLength = outPoint - inPoint;
         long startVideoTimestamp = -1;
         for (int i = 0; i < scsInOutLength; i++)
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

               long videoTimestampMs = Math.round((currentVideoTimestamp - startVideoTimestamp) / 1000.0);
               LogTools.info("Current timestamp: %d  Writing frame %.3f Frequency %.3f".formatted(currentVideoTimestamp,
                                                                                                  videoTimestampMs / 1000.0,
                                                                                                  Conversions.secondsToHertz(period)));
               for (RobotSide side : RobotSide.values)
               {
                  ffmpegRecorders.get(side).writeFrame(zedSVOScrubber, videoTimestampMs);
               }

               ++length;
            }
         }

         LogTools.info("Wrote episode with %d frames.".formatted(length));
         for (RobotSide side : RobotSide.values)
         {
            ffmpegRecorders.get(side).close();
         }

         writeMetadataToFilesystem.run();

      }, getClass().getSimpleName());
   }

   public void appendJsonFileLine()
   {
      LeRobotDatasetTools.appendLine(episodesJsonlPath, JSONFileTools.getAsSingleLine(node ->
      {
         node.put("episode_index", episodeIndex);
         ArrayNode tasksArray = node.putArray("tasks");
         tasksArray.add(taskName);
         node.put("length", length);
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
      return length;
   }
}
