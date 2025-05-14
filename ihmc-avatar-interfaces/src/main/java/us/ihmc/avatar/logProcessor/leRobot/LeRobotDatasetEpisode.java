package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.jerolba.carpet.CarpetReader;
import com.jerolba.carpet.FieldMatchingStrategy;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

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
   private float fps;

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

   public void startGeneratingEpisode(SCS2LogSessionWithVideo session, Runnable writeMetaJson)
   {
      YoLong yoTimestamp = session.getLogDataReader().getTimestamp();

      int inPoint = session.getBufferProperties().getInPoint();
      int outPoint = session.getBufferProperties().getOutPoint();
      double sessionDTSeconds = session.getSessionDTSeconds();
      LogTools.info("Generating episode {}: dt: {}", episodeName, sessionDTSeconds);

      ThreadTools.startAsDaemon(() ->
      {
         session.submitBufferIndexRequestAndWait(inPoint);

         ZEDSVOScrubber zedSVOScrubber = session.getZedSVOScrubbers().get(0);
         fps = zedSVOScrubber.getFps();

         SideDependentList<LeRobotDatasetVideoWriter> ffmpegRecorders = new SideDependentList<>();
         for (RobotSide side : RobotSide.values)
         {
            ffmpegRecorders.put(side, new LeRobotDatasetVideoWriter(side, zedVideoDirs.get(side).resolve(episodeName + ".mp4"), zedSVOScrubber));
         }

         LeRobotDatasetDataWriter dataWriter = new LeRobotDatasetDataWriter(episodeIndex, datasetLengthSoFar, session.getRootRegistry());
         LeRobotDatasetEpisodeStatistics statistics = new LeRobotDatasetEpisodeStatistics();

         LogTools.info("Traversing buffer");
         length = 0L;
         long startVideoTimestamp = -1;
         long lastVideoTimestamp = -1;

         while (true)
         {
            int previousIndex = session.getBufferProperties().getCurrentIndex();

            session.playbackTick();

            if (session.getBufferProperties().getCurrentIndex() < previousIndex)
               break;

            System.out.printf("\rTraversing buffer: %d -> %d / %d", inPoint, session.getBufferProperties().getCurrentIndex(), outPoint);

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
//               LogTools.info("Current timestamp: %d  Writing frame %.3f Frequency %.3f".formatted(currentVideoTimestamp,
//                                                                                                  videoTimestampMicros / 1000.0,
//                                                                                                  Conversions.secondsToHertz(period)));
               for (RobotSide side : RobotSide.values)
               {
                  ffmpegRecorders.get(side).writeFrame(videoTimestampMicros, statistics);
               }

               dataWriter.addFrame(videoTimestampMicros, length, statistics);

               ++length;
            }
         }
         System.out.println();

         LogTools.info("Wrote episode with %d frames.".formatted(length));
         for (RobotSide side : RobotSide.values)
         {
            ffmpegRecorders.get(side).close();
         }

         Path parquetPath = dataChunk0Path.resolve(episodeName + ".parquet");
         dataWriter.writeFile(parquetPath);

         writeEpisodeJsonlLine();
         writeMetaJson.run();

         statistics.calculate();
         writeStatsJsonlLine(statistics);

      }, getClass().getSimpleName());
   }

   public void writeEpisodeJsonlLine()
   {
      LeRobotDatasetTools.appendLine(episodesJsonlPath, JSONFileTools.getAsSingleLine(node ->
      {
         node.put("episode_index", episodeIndex);
         ArrayNode tasksArray = node.putArray("tasks");
         tasksArray.add(taskName);
         node.put("length", length);
      }));
   }

   private void writeStatsJsonlLine(LeRobotDatasetEpisodeStatistics statistics)
   {
      LeRobotDatasetTools.appendLine(episodeStatsJsonlPath, JSONFileTools.getAsSingleLine(node ->
      {
         node.put("episode_index", episodeIndex);
         ObjectNode stats = node.putObject("stats");

         statistics.writeJson(stats, zedVideoDirs);
      }));
   }

   public void readDataAndWriteStatisticsJsonlLine()
   {
      LeRobotDatasetEpisodeStatistics statistics = new LeRobotDatasetEpisodeStatistics();

      for (RobotSide side : RobotSide.values)
      {
         Path videoPath = zedVideoDirs.get(side).resolve(episodeName + ".mp4");
         LogTools.info("Reading video from: %s".formatted(videoPath));
         LeRobotDatasetVideoReader videoReader = new LeRobotDatasetVideoReader(videoPath);

         Mat bgrMat = videoReader.readFrame();
         while (bgrMat != null)
         {
//            LogTools.info("Read frame at timestamp %d : mat: %s".formatted(videoReader.getCurrentTimestamp(), bgrMat));

            statistics.submitFrame(side, bgrMat);

            bgrMat = videoReader.readFrame();
         }

         videoReader.close();
      }
      
      Path parquetPath = dataChunk0Path.resolve(episodeName + ".parquet");
      LogTools.info("Reading parquet data from: %s".formatted(parquetPath));

      try
      {
         File parquetFile = parquetPath.toFile();
         CarpetReader<LeRobotEpisodeRecord> carpetReader = new CarpetReader<>(parquetFile, LeRobotEpisodeRecord.class)
               .withFieldMatchingStrategy(FieldMatchingStrategy.SNAKE_CASE);

         List<LeRobotEpisodeRecord> records = carpetReader.toList();
         LogTools.info("Read %d records from parquet file".formatted(records.size()));

         for (LeRobotEpisodeRecord record : records)
         {
            statistics.processParquetRecord(record);
         }
      }
      catch (IOException e)
      {
         LogTools.error("Failed to read parquet file: " + e.getMessage());
      }

      statistics.calculate();
      writeStatsJsonlLine(statistics);
   }

   public String getEpisodeName()
   {
      return episodeName;
   }

   public int getLength()
   {
      return (int) length;
   }

   public float getFps()
   {
      return fps;
   }
}
