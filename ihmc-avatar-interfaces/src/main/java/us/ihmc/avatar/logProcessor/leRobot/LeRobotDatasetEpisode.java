package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.jerolba.carpet.CarpetReader;
import com.jerolba.carpet.CarpetWriter;
import com.jerolba.carpet.ColumnNamingStrategy;
import com.jerolba.carpet.FieldMatchingStrategy;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.tools.io.JSONFileTools;

import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Represents a single task demonstration.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 */
public class LeRobotDatasetEpisode
{
   private final LeRobotDataset dataset;
   private int episodeIndex;
   private String episodeName;
   private final String taskName;
   private final HumanoidJointNameMap jointMap;
   private final HumanoidRobotSensorInformation sensorInformation;

   private SCS2LogSessionWithVideo session;
   private long startVideoTimestamp;
   private long lastVideoTimestamp;
   private double episodeFrameTimestamp; // important: acrue using double precision
   private SideDependentList<LeRobotDatasetVideoWriter> ffmpegRecorders;
   private LeRobotDatasetDataVariables dataVariables;
   private LeRobotDatasetEpisodeStatistics statistics;
   private final List<LeRobotEpisodeRecord> records = new ArrayList<>();

   /** Load from existing dataset JSON. */
   public LeRobotDatasetEpisode(LeRobotDataset dataset, JsonNode lineRoot, HumanoidJointNameMap jointMap, HumanoidRobotSensorInformation sensorInformation)
   {
      this(dataset, lineRoot.get("episode_index").intValue(), lineRoot.get("tasks").get(0).textValue(), jointMap, sensorInformation);

      statistics = new LeRobotDatasetEpisodeStatistics();
   }

   public LeRobotDatasetEpisode(LeRobotDataset dataset,
                                int episodeIndex,
                                String taskName,
                                HumanoidJointNameMap jointMap,
                                HumanoidRobotSensorInformation sensorInformation)
   {
      this.dataset = dataset;
      this.episodeIndex = episodeIndex;
      this.taskName = taskName;
      this.jointMap = jointMap;
      this.sensorInformation = sensorInformation;

      episodeName = "episode_%06d".formatted(episodeIndex);
   }

   public void reindex(int newIndex)
   {
      episodeIndex = newIndex;
      episodeName = "episode_%06d".formatted(episodeIndex);

      int framesInPreceedingEpisodes = 0;
      for (int i = 0; i < episodeIndex; i++)
         framesInPreceedingEpisodes += dataset.getEpisodes().get(i).getLength();

      List<LeRobotEpisodeRecord> previousRecords = new ArrayList<>(records);
      records.clear();
      for (LeRobotEpisodeRecord record : previousRecords)
      {
         records.add(new LeRobotEpisodeRecord(record.state(),
                                              record.action(),
                                              episodeIndex,
                                              record.frameIndex(),
                                              record.timestamp(),
                                              record.logPosition(),
                                              record.logName(),
                                              record.nextDone(),
                                              framesInPreceedingEpisodes + record.frameIndex(),
                                              record.taskIndex()));
      }
   }

   public void generateFromActiveBuffer(SCS2LogSessionWithVideo session)
   {
      initializeEpisode(session);

      int inPoint = session.getBufferProperties().getInPoint();
      int outPoint = session.getBufferProperties().getOutPoint();
      session.submitBufferIndexRequestAndWait(inPoint);

      ThreadTools.startAThread(() ->
      {
         int bufferIndex = inPoint;
         do
         {
            processFrame();

            ++bufferIndex;
            if (outPoint < inPoint && bufferIndex >= session.getBufferProperties().getSize())
               bufferIndex = 0;

            if (bufferIndex > outPoint)
               break;

            session.submitBufferIndexRequestAndWait(bufferIndex);
         }
         while (true);

         finalizeEpisodeGeneration();
      }, "AddEpisode");
   }

   public void initializeEpisode(SCS2LogSessionWithVideo session)
   {
      this.session = session;

      records.clear();

      double sessionDTSeconds = session.getSessionDTSeconds();
      LogTools.info("Generating episode {}: dt: {}", episodeName, sessionDTSeconds);

      ffmpegRecorders = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         ffmpegRecorders.put(side, new LeRobotDatasetVideoWriter(dataset.getFps(), side, dataset.getZedVideoDirs().get(side).resolve(episodeName + ".mp4")));

      dataVariables = new LeRobotDatasetDataVariables(this, session, jointMap, sensorInformation);
      statistics = new LeRobotDatasetEpisodeStatistics();

      startVideoTimestamp = -1;
      lastVideoTimestamp = -1;
      episodeFrameTimestamp = 0.0;
   }

   public void processFrame()
   {
      int loadedIndex = session.getBufferProperties().getCurrentIndex();
      int inPoint = session.getBufferProperties().getInPoint();
      int outPoint = session.getBufferProperties().getOutPoint();
      long timestamp = session.getLogDataReader().getTimestamp().getLongValue();
      System.out.printf("\rIndex: %d  In: %d  Out: %d  Time: %d", loadedIndex, inPoint, outPoint, timestamp);

      ZEDSVOScrubber zedSVOScrubber = session.getZedSVOScrubbers().get(0);
      synchronized (zedSVOScrubber)
      {
         zedSVOScrubber.scrub(timestamp);
         long currentVideoTimestamp = zedSVOScrubber.getTimestampScrubber().getCurrentVideoTimestamp();

         if (startVideoTimestamp < 0)
            startVideoTimestamp = currentVideoTimestamp;

         long nanosSinceLastFrame = currentVideoTimestamp - lastVideoTimestamp;
         long minNanosBetweenFrames = Conversions.secondsToNanoseconds(UnitConversions.hertzToSeconds(dataset.getFps()) * 0.75);
         if (nanosSinceLastFrame > minNanosBetweenFrames) // Take frames no faster than 133% desired FPS
         {
            lastVideoTimestamp = currentVideoTimestamp;

            for (RobotSide side : RobotSide.values)
               ffmpegRecorders.get(side).writeFrame(statistics, zedSVOScrubber);

            dataVariables.addFrame(episodeFrameTimestamp,
                                   statistics,
                                   session.getLogDataReader().getCurrentLogPosition(),
                                   session.getLogDataReader().getLogDirectory().getName());

            episodeFrameTimestamp += UnitConversions.hertzToSeconds(Math.round(dataset.getFps())); // lerobot rounds fps to integer
         }
      }
   }

   public void finalizeEpisodeGeneration()
   {
      System.out.println();

      LogTools.info("Wrote episode with %d frames.".formatted(records.size()));
      for (RobotSide side : RobotSide.values)
      {
         ffmpegRecorders.get(side).close();
      }

      writeParquetData();

      writeEpisodeJsonlLine();
      dataset.writeMetaJson();

      statistics.calculate();
      writeStatsJsonlLine(statistics);
   }

   public void writeEpisodeJsonlLine()
   {
      LeRobotDatasetTools.appendLine(dataset.getEpisodesJsonlPath(), JSONFileTools.getAsSingleLine(node ->
      {
         node.put("episode_index", episodeIndex);
         ArrayNode tasksArray = node.putArray("tasks");
         tasksArray.add(taskName);
         node.put("length", records.size());
      }));
   }

   private void writeStatsJsonlLine(LeRobotDatasetEpisodeStatistics statistics)
   {
      LeRobotDatasetTools.appendLine(dataset.getEpisodeStatsJsonlPath(), JSONFileTools.getAsSingleLine(node ->
      {
         node.put("episode_index", episodeIndex);
         ObjectNode stats = node.putObject("stats");
         statistics.writeJson(stats, dataset.getZedVideoDirs());
      }));
   }

   public void readDataAndWriteStatisticsJsonlLine()
   {
      LeRobotDatasetEpisodeStatistics statistics = new LeRobotDatasetEpisodeStatistics();

      for (RobotSide side : RobotSide.values)
      {
         Path videoPath = dataset.getZedVideoDirs().get(side).resolve(episodeName + ".mp4");
         LogTools.info("Reading video from: %s".formatted(videoPath));
         LeRobotDatasetVideoReader videoReader = new LeRobotDatasetVideoReader(videoPath);

         Mat bgrMat = videoReader.readFrame();
         while (bgrMat != null)
         {
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
      Path parquetPath = dataset.getDataChunk0Path().resolve(episodeName + ".parquet");
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
      Path parquetPath = dataset.getDataChunk0Path().resolve(episodeName + ".parquet");
      LogTools.info("Writing parquet data to: %s".formatted(parquetPath));

      // Mark the last frame
      LeRobotEpisodeRecord last = records.get(records.size() - 1);
      records.set(records.size() - 1, new LeRobotEpisodeRecord(last.state(),
                                                               last.action(),
                                                               last.episodeIndex(),
                                                               last.frameIndex(),
                                                               last.timestamp(),
                                                               last.logPosition(),
                                                               last.logName(),
                                                               true, // <-- Main thing we're doing here
                                                               last.index(),
                                                               last.taskIndex()));

      try
      {
         OutputStream outputStream = Files.newOutputStream(parquetPath);
         CarpetWriter<LeRobotEpisodeRecord> writer = new CarpetWriter.Builder<>(outputStream, LeRobotEpisodeRecord.class)
               .withColumnNamingStrategy(ColumnNamingStrategy.SNAKE_CASE).build();

         for (LeRobotEpisodeRecord record : records)
            writer.write(record);

         writer.close();
         outputStream.close();
      }
      catch (Exception e)
      {
         DefaultExceptionHandler.MESSAGE_AND_STACKTRACE.handleException(e);
      }
   }

   public LeRobotDataset getDataset()
   {
      return dataset;
   }

   public List<LeRobotEpisodeRecord> getRecords()
   {
      return records;
   }

   public LeRobotDatasetEpisodeStatistics getStatistics()
   {
      return statistics;
   }

   public int getEpisodeIndex()
   {
      return episodeIndex;
   }

   public String getEpisodeName()
   {
      return episodeName;
   }

   public int getLength()
   {
      return records.size();
   }
}
