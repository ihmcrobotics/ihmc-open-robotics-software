package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.JsonNode;
import com.jerolba.carpet.CarpetReader;
import com.jerolba.carpet.CarpetWriter;
import com.jerolba.carpet.FieldMatchingStrategy;
import org.apache.commons.lang3.mutable.MutableBoolean;
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

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

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
      statistics = new LeRobotDatasetEpisodeStatistics();
   }

   public void reindex(int newIndex, int datasetIndexOffset)
   {
      episodeIndex = newIndex;
      episodeName = "episode_%06d".formatted(episodeIndex);
      records.replaceAll(record -> record.reindex(episodeIndex, datasetIndexOffset + record.frameIndex()));
   }

   public BooleanSupplier generateFromActiveBuffer(SCS2LogSessionWithVideo session)
   {
      initializeEpisode(session);

      int inPoint = session.getBufferProperties().getInPoint();
      int outPoint = session.getBufferProperties().getOutPoint();
      System.out.println("InPoint: " + inPoint);
      System.out.println("OutPoint: " + outPoint);
      session.submitBufferIndexRequestAndWait(inPoint);

      MutableBoolean stillGoing = new MutableBoolean(true);
      ThreadTools.startAThread(() ->
      {
         try
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
         }
         finally
         {
            stillGoing.setValue(false);
         }
      }, "AddEpisode");
      return stillGoing::booleanValue;
   }

   public void initializeEpisode(SCS2LogSessionWithVideo session)
   {
      this.session = session;

      records.clear();

      double sessionDTSeconds = session.getSessionDTSeconds();
      LogTools.info("Generating episode {}: dt: {}", episodeName, sessionDTSeconds);

      if (dataset.isHasVideo())
      {
         ffmpegRecorders = new SideDependentList<>();
         for (RobotSide side : RobotSide.values)
            ffmpegRecorders.put(side, new LeRobotDatasetVideoWriter(dataset.getFps(), side, dataset.getZedVideoDirs().get(side).resolve(episodeName + ".mp4")));
      }

      dataVariables = new LeRobotDatasetDataVariables(this, session, jointMap, sensorInformation);
      statistics = new LeRobotDatasetEpisodeStatistics();

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

      long minNanosBetweenFrames = Conversions.secondsToNanoseconds(UnitConversions.hertzToSeconds(dataset.getFps()) * 0.75);
      if (dataset.isHasVideo())
      {
         ZEDSVOScrubber zedSVOScrubber = session.getZedSVOScrubbers().get(0);
         synchronized (zedSVOScrubber)
         {
            zedSVOScrubber.scrub(timestamp);
            long currentVideoTimestamp = zedSVOScrubber.getTimestampScrubber().getCurrentVideoTimestamp();
            sampleFrame(currentVideoTimestamp, minNanosBetweenFrames, zedSVOScrubber);
         }
      }
      else
         sampleFrame(timestamp, minNanosBetweenFrames, null);
   }

   private void sampleFrame(long timestamp, long minNanosBetweenFrames, ZEDSVOScrubber zedSVOScrubber)
   {
      if ((lastVideoTimestamp >= 0 || zedSVOScrubber != null) && timestamp - lastVideoTimestamp <= minNanosBetweenFrames)
         return;

      lastVideoTimestamp = timestamp;
      if (zedSVOScrubber != null)
      {
         for (RobotSide side : RobotSide.values)
            ffmpegRecorders.get(side).writeFrame(statistics, zedSVOScrubber);
      }
      dataVariables.addFrame(episodeFrameTimestamp,
                             statistics,
                             session.getLogDataReader().getCurrentLogPosition(),
                             session.getLogDataReader().getLogDirectory().getName());
      episodeFrameTimestamp += UnitConversions.hertzToSeconds(Math.round(dataset.getFps())); // lerobot rounds fps to integer
   }

   public void finalizeEpisodeGeneration()
   {
      System.out.println();
      LogTools.info("Processed episode with %d frames.".formatted(records.size()));

      if (dataset.isHasVideo())
      {
         for (RobotSide side : RobotSide.values)
            ffmpegRecorders.get(side).close();
      }

      // Mark last frame as done
      if (!records.isEmpty())
         records.set(records.size() - 1, records.get(records.size() - 1).markDone());

      statistics.calculate();
      // In v3.0 format, per-episode files are not written here; call dataset.finalizeDataset() when done.
   }

   public void loadParquetData(Path parquetPath)
   {
      LogTools.info("Reading parquet data from: %s".formatted(parquetPath));
      try
      {
         CarpetReader<LeRobotParquetRecord> carpetReader = new CarpetReader<>(parquetPath.toFile(), LeRobotParquetRecord.class)
               .withFieldMatchingStrategy(FieldMatchingStrategy.SNAKE_CASE)
               .withFailOnMissingColumn(false);
         records.clear();
         for (LeRobotParquetRecord pr : carpetReader)
            records.add(pr.toEpisodeRecord());
         LogTools.info("Read %d records from parquet file".formatted(records.size()));
      }
      catch (Exception e)
      {
         LogTools.error("Failed to read parquet file: " + e.getMessage());
      }
   }

   /** Writes this episode's records to the given writer (caller manages open/close). */
   public void writeRecordsToStream(CarpetWriter<LeRobotParquetRecord> writer) throws Exception
   {
      for (LeRobotEpisodeRecord r : records)
         writer.write(new LeRobotParquetRecord(r));
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

   /** Rebuilds statistics after loading records from the combined v3 parquet file. */
   public void recomputeStatistics()
   {
      statistics = new LeRobotDatasetEpisodeStatistics();
      for (LeRobotEpisodeRecord record : records)
         statistics.processParquetRecord(record);
      statistics.calculate();
   }

   public int getEpisodeIndex()
   {
      return episodeIndex;
   }

   public String getEpisodeName()
   {
      return episodeName;
   }

   public String getTaskName()
   {
      return taskName;
   }

   public int getLength()
   {
      return records.size();
   }
}
