package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.jerolba.carpet.CarpetReader;
import com.jerolba.carpet.FieldMatchingStrategy;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.UnitConversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;
import us.ihmc.tools.io.JSONFileTools;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * Represents a single task demonstration.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 */
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

   private SCS2LogSessionWithVideo session;
   private Runnable writeMetaJson;
   private Consumer<Runnable> frameProcessingQueue;
   private boolean usePerfectTimestamps;
   private long startVideoTimestamp;
   private long lastVideoTimestamp;
   private double episodeFrameTimestampMicros;
   private SideDependentList<LeRobotDatasetVideoWriter> ffmpegRecorders;
   private LeRobotDatasetDataWriter dataWriter;
   private LeRobotDatasetEpisodeStatistics statistics;
   private final List<LeRobotEpisodeRecord> records = new ArrayList<>();

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

   public void startGeneratingEpisode(SCS2LogSessionWithVideo session, Runnable writeMetaJson, Consumer<Runnable> frameProcessingQueue, boolean usePerfectTimestamps)
   {
      this.session = session;
      this.writeMetaJson = writeMetaJson;
      this.frameProcessingQueue = frameProcessingQueue;
      this.usePerfectTimestamps = usePerfectTimestamps;

      records.clear();

      int inPoint = session.getBufferProperties().getInPoint();
      double sessionDTSeconds = session.getSessionDTSeconds();
      LogTools.info("Generating episode {}: dt: {}", episodeName, sessionDTSeconds);

      session.submitBufferIndexRequestAndWait(inPoint);

      ZEDSVOScrubber zedSVOScrubber = session.getZedSVOScrubbers().get(0);
      fps = zedSVOScrubber.getFps();

      ffmpegRecorders = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         ffmpegRecorders.put(side, new LeRobotDatasetVideoWriter(side, zedVideoDirs.get(side).resolve(episodeName + ".mp4"), zedSVOScrubber));
      }

      dataWriter = new LeRobotDatasetDataWriter(episodeIndex, datasetLengthSoFar, session);
      statistics = new LeRobotDatasetEpisodeStatistics();

      length = 0L;
      startVideoTimestamp = -1;
      lastVideoTimestamp = -1;
      episodeFrameTimestampMicros = -Math.round(UnitConversions.hertzToSeconds(fps));

      frameProcessingQueue.accept(this::processFrame);
   }

   private void processFrame()
   {
      int previousIndex = session.getBufferProperties().getCurrentIndex();

      session.playbackTick();

      int currentBufferIndex = session.getBufferProperties().getCurrentIndex();
      if (currentBufferIndex < previousIndex)
      {
         frameProcessingQueue.accept(this::finalizeEpisodeGeneration);
         return;
      }
      else
      {
         frameProcessingQueue.accept(this::processFrame);
      }

      int inPoint = session.getBufferProperties().getInPoint();
      int outPoint = session.getBufferProperties().getOutPoint();
      System.out.printf("\rTraversing buffer: %d -> %d / %d", inPoint, currentBufferIndex, outPoint);

      long timestamp = session.getLogDataReader().getTimestamp().getLongValue();
      ZEDSVOScrubber zedSVOScrubber = session.getZedSVOScrubbers().get(0);
      zedSVOScrubber.scrub(timestamp);
      long currentVideoTimestamp = zedSVOScrubber.getTimestampScrubber().getCurrentVideoTimestamp();
      if (startVideoTimestamp < 0)
         startVideoTimestamp = currentVideoTimestamp;

      if (currentVideoTimestamp > lastVideoTimestamp) // Write only when a new frame is available
      {
         double frequency = lastVideoTimestamp >= 0 ?
               Conversions.secondsToHertz(Conversions.nanosecondsToSeconds(currentVideoTimestamp - lastVideoTimestamp)): Double.NaN;

         lastVideoTimestamp = currentVideoTimestamp;

         if (usePerfectTimestamps)
         {
            int round = Math.round(fps); // lerobot rounds fps to integer
            double seconds = UnitConversions.hertzToSeconds(round);
            double micros = seconds * 1000000.0;
            episodeFrameTimestampMicros += micros; // important: acrue using double precision
         }
         else
         {
            episodeFrameTimestampMicros = Math.round((currentVideoTimestamp - startVideoTimestamp) / 1000.0);
         }

         long roundedTimestamp = Math.round(episodeFrameTimestampMicros);

//         LogTools.info("Current timestamp: %.3f  Writing frame %d Frequency %.3f"
//                             .formatted(Conversions.nanosecondsToSeconds(currentVideoTimestamp),
//                                        roundedTimestamp,
//                                        frequency));
         for (RobotSide side : RobotSide.values)
         {
            ffmpegRecorders.get(side).writeFrame(roundedTimestamp, statistics);
         }

         records.add(dataWriter.addFrame(roundedTimestamp, length, statistics, session.getLogDataReader().getCurrentLogPosition()));

         ++length;
      }
   }

   private void finalizeEpisodeGeneration()
   {
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

      loadParquetData();
      for (LeRobotEpisodeRecord record : records)
      {
         statistics.processParquetRecord(record);
      }

      statistics.calculate();
      writeStatsJsonlLine(statistics);
   }

   public void loadParquetData()
   {
      Path parquetPath = dataChunk0Path.resolve(episodeName + ".parquet");
      LogTools.info("Reading parquet data from: %s".formatted(parquetPath));

      try
      {
         File parquetFile = parquetPath.toFile();
         CarpetReader<LeRobotEpisodeRecord> carpetReader = new CarpetReader<>(parquetFile, LeRobotEpisodeRecord.class)
               .withFieldMatchingStrategy(FieldMatchingStrategy.SNAKE_CASE)
               .withFailOnMissingColumn(false);

         records.clear();
         records.addAll(carpetReader.toList());
         LogTools.info("Read %d records from parquet file".formatted(records.size()));
      }
      catch (IOException e)
      {
         LogTools.error("Failed to read parquet file: " + e.getMessage());
      }
   }

   public void writeParquetData()
   {
      Path parquetPath = dataChunk0Path.resolve(episodeName + ".parquet");
      LogTools.info("Writing parquet data to: %s".formatted(parquetPath));
      LeRobotDatasetDataWriter.writeParquetFile(parquetPath, records);
   }

   public List<LeRobotEpisodeRecord> getRecords()
   {
      return records;
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
