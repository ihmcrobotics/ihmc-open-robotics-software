package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.jerolba.carpet.CarpetReader;
import com.jerolba.carpet.CarpetWriter;
import com.jerolba.carpet.ColumnNamingStrategy;
import com.jerolba.carpet.FieldMatchingStrategy;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commons.thread.ThreadTools;

/**
 * Represents a LeRobot v3.0 dataset for generating datasets from IHMC logs.
 * <p>
 * v3.0 format differences from v2.1:
 * - All episode data is concatenated into a single data/chunk-000/file-000.parquet
 * - All episode videos are concatenated into videos/{video_key}/chunk-000/file-000.mp4
 * - Episode metadata is in meta/episodes/chunk-000/file-000.parquet
 * - Global stats in meta/stats.json; tasks in meta/tasks.parquet
 * - Call {@link #finalizeDataset()} after all episodes have been added.
 */
public class LeRobotDataset
{
   public static boolean XYZ_RIGHT_ONLY = Boolean.parseBoolean(System.getProperty("xyz.right.only", "true"));
   /** Legacy live-policy size retained for compatibility with {@code VLAUpdateThread} on develop. */
   public static int STATE_SIZE = XYZ_RIGHT_ONLY ? 3 : LeRobotDatasetDataVariables.STATE_SIZE;
   /** Full state width written by the log converter. */
   public static final int RECORDED_STATE_SIZE = LeRobotDatasetDataVariables.STATE_SIZE;
   public static final int ACTION_SIZE = LeRobotDatasetDataVariables.ACTION_SIZE;

   private final HumanoidJointNameMap jointMap;
   private final HumanoidRobotSensorInformation sensorInformation;
   private final String name;
   private final Path directory;
   private final Path dataPath;
   private final Path metaPath;
   private final Path videosPath;
   private final Path dataChunk0Path;
   private final Path dataFilePath;           // data/chunk-000/file-000.parquet
   private final Path metaEpisodesChunkPath;  // meta/episodes/chunk-000/
   private final Path statsJsonPath;          // meta/stats.json
   private final Path tasksParquetPath;       // meta/tasks.parquet
   private final Path infoJsonPath;           // meta/info.json
   /** videos/{video_key}/chunk-000/ — temp episode MP4s are written here; final file-000.mp4 also lives here. */
   private final SideDependentList<Path> zedVideoDirs = new SideDependentList<>();

   private float fps = 30.0f;
   private boolean hasVideo = false;
   private final List<String> taskNames = new ArrayList<>();
   private final List<LeRobotDatasetEpisode> episodes = new ArrayList<>();
   private long totalFrames = 0L;

   public LeRobotDataset(Path directory, HumanoidJointNameMap jointMap, HumanoidRobotSensorInformation sensorInformation)
   {
      this.directory = directory;
      this.jointMap = jointMap;
      this.sensorInformation = sensorInformation;

      name = directory.getFileName().toString();
      dataPath = directory.resolve("data");
      metaPath = directory.resolve("meta");
      videosPath = directory.resolve("videos");
      dataChunk0Path = dataPath.resolve("chunk-000");
      dataFilePath = dataChunk0Path.resolve("file-000.parquet");
      metaEpisodesChunkPath = metaPath.resolve("episodes/chunk-000");
      statsJsonPath = metaPath.resolve("stats.json");
      tasksParquetPath = metaPath.resolve("tasks.parquet");
      infoJsonPath = metaPath.resolve("info.json");

      // v3.0 video layout: videos/{video_key}/chunk-000/
      for (RobotSide side : RobotSide.values)
         zedVideoDirs.put(side, videosPath.resolve("observation.images.cam_zed_" + side.getLowerCaseName() + "/chunk-000"));
   }

   public void mkdirs()
   {
      FileTools.ensureDirectoryExists(directory, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(dataPath, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(metaPath, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(dataChunk0Path, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(videosPath, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(metaEpisodesChunkPath, DefaultExceptionHandler.PRINT_MESSAGE);
      for (RobotSide side : RobotSide.values)
         FileTools.ensureDirectoryExists(zedVideoDirs.get(side), DefaultExceptionHandler.PRINT_MESSAGE);

      FileTools.ensureFileExists(infoJsonPath, DefaultExceptionHandler.PRINT_MESSAGE);
   }

   public void loadData()
   {
      JSONFileTools.load(infoJsonPath, rootNode -> {
         fps = rootNode.get("fps").floatValue();
         hasVideo = rootNode.has("video_path");
      });

      taskNames.clear();
      if (Files.exists(tasksParquetPath))
      {
         ExceptionTools.handle(() ->
         {
            CarpetReader<LeRobotTaskRecord> reader = new CarpetReader<>(tasksParquetPath.toFile(), LeRobotTaskRecord.class)
                  .withFieldMatchingStrategy(FieldMatchingStrategy.SNAKE_CASE)
                  .withFailOnMissingColumn(false);
            reader.forEach(r -> taskNames.add(r.task()));
         }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      }

      episodes.clear();
      totalFrames = 0L;

      Path episodesParquetPath = metaEpisodesChunkPath.resolve("file-000.parquet");
      if (Files.exists(episodesParquetPath))
      {
         ExceptionTools.handle(() ->
         {
            CarpetReader<LeRobotEpisodeMetadataRecord> reader =
                  new CarpetReader<>(episodesParquetPath.toFile(), LeRobotEpisodeMetadataRecord.class)
                        .withFieldMatchingStrategy(FieldMatchingStrategy.SNAKE_CASE)
                        .withFailOnMissingColumn(false);
            for (LeRobotEpisodeMetadataRecord meta : reader)
            {
               String taskName = meta.tasks().isEmpty() ? "" : meta.tasks().get(0);
               LeRobotDatasetEpisode episode = new LeRobotDatasetEpisode(this, (int) meta.episodeIndex(), taskName, jointMap, sensorInformation);
               episodes.add(episode);
               totalFrames += meta.length();
            }
         }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      }

      // Read the combined parquet and distribute records by episode_index
      if (Files.exists(dataFilePath) && !episodes.isEmpty())
      {
         ExceptionTools.handle(() ->
         {
            CarpetReader<LeRobotParquetRecord> reader = new CarpetReader<>(dataFilePath.toFile(), LeRobotParquetRecord.class)
                  .withFieldMatchingStrategy(FieldMatchingStrategy.SNAKE_CASE)
                  .withFailOnMissingColumn(false);
            for (LeRobotParquetRecord pr : reader)
            {
               int epIdx = (int) pr.episodeIndex();
               if (epIdx >= 0 && epIdx < episodes.size())
                  episodes.get(epIdx).getRecords().add(pr.toEpisodeRecord());
            }
            for (LeRobotDatasetEpisode episode : episodes)
               episode.recomputeStatistics();
         }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      }
   }

   public BooleanSupplier addEpisode(String taskName, SCS2LogSessionWithVideo session)
   {
      hasVideo = !session.getZedSVOScrubbers().isEmpty();
      return createEpisode(taskName).generateFromActiveBuffer(session);
   }

   private LeRobotDatasetEpisode createEpisode(String taskName)
   {
      ensureTaskName(taskName);
      LeRobotDatasetEpisode episode = new LeRobotDatasetEpisode(this, episodes.size(), taskName, jointMap, sensorInformation);
      episodes.add(episode);
      return episode;
   }

   public BooleanSupplier addEpisodesAutomatically(String taskName, int taskID, SCS2LogSessionWithVideo session, BooleanSupplier keepGoing)
   {
      hasVideo = !session.getZedSVOScrubbers().isEmpty();
      ensureTaskName(taskName);

      MutableBoolean stillGoing = new MutableBoolean(true);
      YoRegistry rootReg = session.getRootRegistry();
      String rootName = rootReg.getName();
      String kstModule = null;
      for (YoRegistry child : rootReg.getChildren())
      {
         String path = rootName + "." + child.getName();
         kstModule = LeRobotDatasetTools.findRegistry(rootReg, path, "IKStreamingRTThread");
         if (kstModule != null)
            break;
      }
      if (kstModule == null)
      {
         LogTools.error("Could not find IKStreamingRTThread registry - Auto Scrub requires KST to be running in the log.");
         return stillGoing::booleanValue;
      }
      String kstStreaming = kstModule + "KinematicsStreamingToolboxController.KSTStreamingState.";
      if (session.getRootRegistry().findVariable(kstStreaming + "demonstrationTaskID") instanceof YoInteger demonstrationTaskID)
         startAutoScrubThread(taskName, () -> demonstrationTaskID.getValue() == taskID, session, keepGoing, stillGoing, 0);
      else
         stillGoing.setValue(false);
      return stillGoing::booleanValue;
   }

   /**
    * Adds episodes by scrubbing the log from the current position, starting an episode when
    * {@code variableName} rises above {@code threshold} and ending it when the value drops back below.
    * For example, {@code q_right_ability_hand_index_q1 > 0.15} marks an episode in progress.
    * Episodes shorter than {@code minEpisodeDuration} seconds are discarded.
    */
   public BooleanSupplier addEpisodesAutomatically(String taskName,
                                                   String variableName,
                                                   double threshold,
                                                   double minEpisodeDuration,
                                                   SCS2LogSessionWithVideo session,
                                                   BooleanSupplier keepGoing)
   {
      hasVideo = !session.getZedSVOScrubbers().isEmpty();
      ensureTaskName(taskName);

      MutableBoolean stillGoing = new MutableBoolean(true);
      YoVariable variable = session.getRootRegistry().findVariable(variableName);
      if (variable == null)
      {
         LogTools.error("Could not find variable {} - cannot auto scrub.", variableName);
         stillGoing.setValue(false);
      }
      else
      {
         int minFrames = (int) Math.round(minEpisodeDuration * fps);
         startAutoScrubThread(taskName, () -> variable.getValueAsDouble() > threshold, session, keepGoing, stillGoing, minFrames);
      }
      return stillGoing::booleanValue;
   }

   private void startAutoScrubThread(String taskName,
                                     BooleanSupplier episodeActive,
                                     SCS2LogSessionWithVideo session,
                                     BooleanSupplier keepGoing,
                                     MutableBoolean stillGoing,
                                     int minFramesPerEpisode)
   {
      ThreadTools.startAThread(() ->
      {
         try
         {
            // Sample the log at ~2x the dataset FPS instead of loading every tick;
            // frame capture and episode boundary detection only need that resolution.
            double sessionDT = session.getSessionDTSeconds();
            int strideTicks = sessionDT > 0.0 ? Math.max(1, (int) (1.0 / (2.0 * fps) / sessionDT)) : 1;
            LogTools.info("Auto scrub sampling every {} log ticks", strideTicks);

            LeRobotDatasetEpisode episode = null;
            int desiredLoadedIndex = Math.max(0, session.getLogDataReader().getCurrentLogPosition() - 1);
            while (keepGoing.getAsBoolean())
            {
               int currentPosition = session.getLogDataReader().getCurrentLogPosition();
               if (currentPosition - 1 == desiredLoadedIndex)
               {
                  if (episodeActive.getAsBoolean())
                  {
                     if (episode == null)
                     {
                        episode = createEpisode(taskName);
                        episode.initializeEpisode(session);
                     }
                     episode.processFrame();
                  }
                  else if (episode != null)
                  {
                     finishEpisode(episode, minFramesPerEpisode);
                     episode = null;
                  }

                  int numberOfEntries = session.getLogDataReader().getNumberOfEntries();
                  if (currentPosition >= numberOfEntries)
                     break;
                  desiredLoadedIndex = Math.min(currentPosition + strideTicks - 1, numberOfEntries - 1);
                  session.submitLogPositionRequest(desiredLoadedIndex);
               }
               else
               {
                  // The session sets the log position at seek and increments it only after the read
                  // completes, so transient positions are visible here. Only resubmit the unchanged
                  // target; recomputing it from a transient position desynchronizes this loop from
                  // the session and no sample ever appears loaded.
                  session.submitLogPositionRequest(desiredLoadedIndex);
               }
               ThreadTools.park(0.000001);
            }

            if (episode != null)
               finishEpisode(episode, minFramesPerEpisode);
         }
         catch (Exception e)
         {
            DefaultExceptionHandler.MESSAGE_AND_STACKTRACE.handleException(e);
         }
         finally
         {
            stillGoing.setValue(false);
         }
      }, "ScrubToNextEpisode");
   }

   private void finishEpisode(LeRobotDatasetEpisode episode, int minFramesPerEpisode)
   {
      episode.finalizeEpisodeGeneration();
      if (episode.getLength() < minFramesPerEpisode)
      {
         LogTools.info("Discarding episode {} ({} frames < {} minimum)", episode.getEpisodeName(), episode.getLength(), minFramesPerEpisode);
         episodes.remove(episode);
         if (hasVideo)
         {
            for (RobotSide side : RobotSide.values)
               ExceptionTools.handle(() -> Files.deleteIfExists(zedVideoDirs.get(side).resolve(episode.getEpisodeName() + ".mp4")),
                                     DefaultExceptionHandler.PRINT_MESSAGE);
         }
      }
   }

   private void ensureTaskName(String taskName)
   {
      if (!taskNames.contains(taskName))
         taskNames.add(taskName);
   }

   /**
    * Finalizes the dataset in v3.0 format: writes the combined parquet, concatenates videos,
    * writes episode metadata, global stats, tasks, and info.json.
    * Call this once after all episodes have been recorded.
    */
   public void finalizeDataset()
   {
      if (episodes.isEmpty())
         throw new IllegalStateException("Cannot finalize an empty LeRobot dataset");
      String python = findPythonWithDependencies();
      if (hasVideo)
         requireCommand("ffmpeg", "-version");

      LogTools.info("Finalizing v3.0 dataset with {} episodes...", episodes.size());
      FileTools.ensureDirectoryExists(metaEpisodesChunkPath, DefaultExceptionHandler.PRINT_MESSAGE);

      writeAllParquetData();
      writeTasksParquet();
      fixParquetFormat(python);
      if (hasVideo)
         concatenateEpisodeVideos();
      writeEpisodesMetadataParquet();
      writeGlobalStatsJson();
      writeMetaJson();

      LogTools.info("Dataset finalization complete: {}", directory);
   }

   private void writeAllParquetData()
   {
      LogTools.info("Writing combined parquet: {}", dataFilePath);
      try (OutputStream outputStream = Files.newOutputStream(dataFilePath))
      {
         CarpetWriter<LeRobotParquetRecord> writer = new CarpetWriter.Builder<>(outputStream, LeRobotParquetRecord.class)
               .withColumnNamingStrategy(ColumnNamingStrategy.SNAKE_CASE).build();
         for (LeRobotDatasetEpisode episode : episodes)
            episode.writeRecordsToStream(writer);
         writer.close();
      }
      catch (Exception e)
      {
         LogTools.error("Failed to write combined parquet: " + e.getMessage());
      }
   }

   /**
    * Post-processes the generated parquet files so they match what lerobot v3 expects:
    * 1. observation.state / action: variable-length list<float> → fixed_size_list<float32>
    * 2. tasks.parquet: {task_index, task} columns → task string as pandas index, task_index as sole column
    */
   private void fixParquetFormat(String python)
   {
      String pythonCode = """
            import pyarrow as pa, pyarrow.parquet as pq, numpy as np, pandas as pd
            from pathlib import Path
            root = Path('%s')

            # 1. Fix data parquet: variable-length list<float> → fixed_size_list<float32>
            for path in root.glob('data/**/*.parquet'):
                t = pq.read_table(str(path))
                changed = False
                for col in ['observation.state', 'action']:
                    if col in t.schema.names and pa.types.is_list(t.schema.field(col).type):
                        arr = np.array(t.column(col).to_pylist(), dtype=np.float32)
                        fsl = pa.FixedSizeListArray.from_arrays(pa.array(arr.ravel(), type=pa.float32()), arr.shape[1])
                        t = t.set_column(t.schema.get_field_index(col), pa.field(col, fsl.type), fsl)
                        changed = True
                if changed:
                    pq.write_table(t, str(path), compression='snappy')
                    print(f'Fixed list types: {path.name}')

            # 2. Fix tasks.parquet: task string must be the pandas index, task_index the only column
            tasks_path = root / 'meta/tasks.parquet'
            if tasks_path.exists():
                df = pd.read_parquet(str(tasks_path))
                if 'task' in df.columns:
                    df = df.set_index('task')[['task_index']]
                    df.to_parquet(str(tasks_path))
                    print(f'Fixed tasks.parquet index')
            """.formatted(directory.toAbsolutePath().toString().replace("\\", "\\\\"));

      try
      {
         LogTools.info("Fixing parquet array types via Python ({})...", python);
         int exitCode = new ProcessBuilder(python, "-c", pythonCode).inheritIO().start().waitFor();
         if (exitCode != 0)
            throw new IllegalStateException("Parquet array type fix failed with exit " + exitCode);
      }
      catch (Exception e)
      {
         throw new IllegalStateException("Could not produce LeRobot-compatible parquet arrays", e);
      }
   }

   private static String findPythonWithDependencies()
   {
      String[] candidates = {System.getProperty("user.home") + "/miniconda3/envs/lerobot/bin/python3", "python3", "python"};
      for (String candidate : candidates)
      {
         if (commandSucceeds(candidate, "-c", "import numpy, pandas, pyarrow"))
            return candidate;
      }
      throw new IllegalStateException("Finalization requires Python with numpy, pandas, and pyarrow");
   }

   private static void requireCommand(String... command)
   {
      if (!commandSucceeds(command))
         throw new IllegalStateException("Required command is unavailable: " + command[0]);
   }

   private static boolean commandSucceeds(String... command)
   {
      try
      {
         return new ProcessBuilder(command).redirectErrorStream(true).start().waitFor() == 0;
      }
      catch (Exception ignored)
      {
         return false;
      }
   }

   private void concatenateEpisodeVideos()
   {
      for (RobotSide side : RobotSide.values)
      {
         Path videoDir = zedVideoDirs.get(side);
         Path concatList = videoDir.resolve("concat_list.txt");
         Path outputVideo = videoDir.resolve("file-000.mp4");

         List<Path> episodeVideos = new ArrayList<>();
         for (LeRobotDatasetEpisode episode : episodes)
         {
            Path tempVideo = videoDir.resolve(episode.getEpisodeName() + ".mp4");
            if (Files.exists(tempVideo))
               episodeVideos.add(tempVideo);
         }

         if (episodeVideos.isEmpty())
         {
            LogTools.warn("No temp episode videos found for {} — skipping video concat", side);
            continue;
         }

         LogTools.info("Concatenating {} episode videos → {}", episodeVideos.size(), outputVideo);

         try
         {
            List<String> concatLines = new ArrayList<>();
            for (Path video : episodeVideos)
               concatLines.add("file '" + video.toAbsolutePath() + "'");
            Files.write(concatList, concatLines);

            ProcessBuilder pb = new ProcessBuilder("ffmpeg", "-y",
                                                   "-f", "concat", "-safe", "0",
                                                   "-i", concatList.toAbsolutePath().toString(),
                                                   "-c", "copy",
                                                   outputVideo.toAbsolutePath().toString());
            pb.inheritIO();
            int exitCode = pb.start().waitFor();
            Files.deleteIfExists(concatList);

            if (exitCode != 0)
               LogTools.error("ffmpeg concat failed (exit {}) for {}", exitCode, side);
            else
            {
               LogTools.info("Video concat succeeded: {}", outputVideo);
               for (Path video : episodeVideos)
                  ExceptionTools.handle(() -> Files.deleteIfExists(video), DefaultExceptionHandler.PRINT_MESSAGE);
            }
         }
         catch (Exception e)
         {
            LogTools.error("Video concat error for {}: {}", side, e.getMessage());
            ExceptionTools.handle(() -> Files.deleteIfExists(concatList), DefaultExceptionHandler.PRINT_MESSAGE);
         }
      }
   }

   private void writeEpisodesMetadataParquet()
   {
      Path episodesParquetPath = metaEpisodesChunkPath.resolve("file-000.parquet");
      LogTools.info("Writing episodes metadata parquet: {}", episodesParquetPath);
      try (OutputStream out = Files.newOutputStream(episodesParquetPath))
      {
         CarpetWriter<LeRobotEpisodeMetadataRecord> writer =
               new CarpetWriter.Builder<>(out, LeRobotEpisodeMetadataRecord.class)
                     .withColumnNamingStrategy(ColumnNamingStrategy.SNAKE_CASE).build();

         long fromIdx = 0L;
         for (LeRobotDatasetEpisode episode : episodes)
         {
            long length = episode.getLength();
            double fromTs = fromIdx / (double) fps;
            double toTs = (fromIdx + length) / (double) fps;
            // chunk/file indices are always 0 — we produce a single shard per camera/data
            writer.write(new LeRobotEpisodeMetadataRecord(episode.getEpisodeIndex(),
                                                          List.of(episode.getTaskName()),
                                                          length,
                                                          fromIdx,
                                                          fromIdx + length,
                                                          0L, 0L,           // data/chunk_index, data/file_index
                                                          0L, 0L, fromTs, toTs,  // cam_zed_left chunk/file/from/to
                                                          0L, 0L, fromTs, toTs));// cam_zed_right chunk/file/from/to
            fromIdx += length;
         }
         writer.close();
      }
      catch (Exception e)
      {
         LogTools.error("Failed to write episodes metadata parquet: " + e.getMessage());
      }
   }

   private void writeGlobalStatsJson()
   {
      LogTools.info("Writing global stats: {}", statsJsonPath);
      LeRobotDatasetEpisodeStatistics globalStats = new LeRobotDatasetEpisodeStatistics();
      for (LeRobotDatasetEpisode episode : episodes)
         globalStats.mergeFrom(episode.getStatistics());
      globalStats.calculate();

      SideDependentList<String> videoFeatureKeys = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         videoFeatureKeys.put(side, hasVideo ? "observation.images.cam_zed_" + side.getLowerCaseName() : null);

      JSONFileTools.save(statsJsonPath, rootNode -> globalStats.writeJson(rootNode, videoFeatureKeys));
   }

   private void writeTasksParquet()
   {
      LogTools.info("Writing tasks parquet: {}", tasksParquetPath);
      try (OutputStream out = Files.newOutputStream(tasksParquetPath))
      {
         CarpetWriter<LeRobotTaskRecord> writer =
               new CarpetWriter.Builder<>(out, LeRobotTaskRecord.class)
                     .withColumnNamingStrategy(ColumnNamingStrategy.SNAKE_CASE).build();
         for (int i = 0; i < taskNames.size(); i++)
            writer.write(new LeRobotTaskRecord(i, taskNames.get(i)));
         writer.close();
      }
      catch (Exception e)
      {
         LogTools.error("Failed to write tasks parquet: " + e.getMessage());
      }
   }

   public void removeEpisodes(boolean[] episodesToRemove)
   {
      ExceptionTools.handle(() ->
      {
         // Delete temp videos for removed episodes, collect remaining
         List<LeRobotDatasetEpisode> remaining = new ArrayList<>();
         for (int i = 0; i < episodesToRemove.length; i++)
         {
            if (episodesToRemove[i])
            {
               if (hasVideo)
               {
                  for (RobotSide side : RobotSide.values)
                     Files.deleteIfExists(zedVideoDirs.get(side).resolve(episodes.get(i).getEpisodeName() + ".mp4"));
               }
            }
            else
            {
               remaining.add(episodes.get(i));
            }
         }

         // Rename temp videos to match new contiguous indices, then reindex
         int datasetIndexOffset = 0;
         for (int newIdx = 0; newIdx < remaining.size(); newIdx++)
         {
            LeRobotDatasetEpisode episode = remaining.get(newIdx);
            int oldIdx = episode.getEpisodeIndex();
            if (oldIdx != newIdx && hasVideo)
            {
               for (RobotSide side : RobotSide.values)
               {
                  Path oldVideo = zedVideoDirs.get(side).resolve("episode_%06d.mp4".formatted(oldIdx));
                  Path newVideo = zedVideoDirs.get(side).resolve("episode_%06d.mp4".formatted(newIdx));
                  if (Files.exists(oldVideo))
                     Files.move(oldVideo, newVideo);
               }
            }
            episode.reindex(newIdx, datasetIndexOffset);
            datasetIndexOffset += episode.getLength();
         }

         episodes.clear();
         episodes.addAll(remaining);

         finalizeDataset();
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public void regenerateAndRewriteMetadata()
   {
      // In v3.0 all metadata is regenerated by finalizeDataset()
      finalizeDataset();
   }

   public void writeParquetData()
   {
      writeAllParquetData();
   }

   public void writeMetaJson()
   {
      JSONFileTools.save(infoJsonPath, rootNode ->
      {
         totalFrames = 0;
         for (LeRobotDatasetEpisode episode : episodes)
            totalFrames += episode.getLength();

         rootNode.put("codebase_version", "v3.0");
         rootNode.put("robot_type", "alex");
         rootNode.put("total_episodes", episodes.size());
         rootNode.put("total_frames", totalFrames);
         rootNode.put("total_tasks", taskNames.size());
         rootNode.put("total_chunks", 1);
         rootNode.put("chunks_size", 1000);
         rootNode.put("fps", fps);
         ObjectNode splits = rootNode.putObject("splits");
         splits.put("train", "0:%d".formatted(episodes.size()));
         rootNode.put("data_path", "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet");
         if (hasVideo)
            rootNode.put("video_path", "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4");

         ObjectNode features = rootNode.putObject("features");
         if (hasVideo)
         {
            for (RobotSide side : RobotSide.values)
            {
               String key = "observation.images.cam_zed_%s".formatted(side.getLowerCaseName());
               ObjectNode cam = features.putObject(key);
               cam.put("dtype", "video");
               cam.putArray("shape").add(480).add(640).add(3);
               cam.putArray("names").add("height").add("width").add("channel");
               cam.put("fps", fps);
               cam.putObject("video_info").put("video.fps", fps)
                                          .put("video.codec", "h264")
                                          .put("video.pix_fmt", "yuv420p")
                                          .put("video.is_depth_map", false)
                                          .put("has_audio", false);
            }
         }

         addVectorFeature(features, "observation.state", RECORDED_STATE_SIZE, LeRobotDatasetDataVariables.getStateFeatureNames(), fps);
         addVectorFeature(features, "action", ACTION_SIZE, LeRobotDatasetDataVariables.getActionFeatureNames(), fps);

         addScalarFeature(features, "episode_index", "int64", fps);
         addScalarFeature(features, "frame_index", "int64", fps);
         addScalarFeature(features, "timestamp", "float32", fps);
         addScalarFeature(features, "next.done", "bool", fps);
         addScalarFeature(features, "index", "int64", fps);
         addScalarFeature(features, "task_index", "int64", fps);
      });
   }

   private static void addVectorFeature(ObjectNode features, String name, int size, List<String> featureNames, float fps)
   {
      ObjectNode node = features.putObject(name);
      node.put("dtype", "float32");
      node.putArray("shape").add(size);
      ArrayNode motors = node.putObject("names").putArray("motors");
      featureNames.forEach(motors::add);
      node.put("fps", fps);
   }

   private static void addScalarFeature(ObjectNode features, String name, String dtype, float fps)
   {
      ObjectNode node = features.putObject(name);
      node.put("dtype", dtype);
      node.putArray("shape").add(1);
      node.putNull("names");
      node.put("fps", fps);
   }

   public String getName()
   {
      return name;
   }

   public Path getDirectory()
   {
      return directory;
   }

   public Path getDataPath()
   {
      return dataPath;
   }

   public Path getMetaPath()
   {
      return metaPath;
   }

   public Path getVideosPath()
   {
      return videosPath;
   }

   public Path getDataChunk0Path()
   {
      return dataChunk0Path;
   }

   public Path getEpisodesJsonlPath()
   {
      // Legacy: no longer used in v3.0 (kept for compilation compatibility)
      return metaPath.resolve("episodes.jsonl");
   }

   public Path getEpisodeStatsJsonlPath()
   {
      // Legacy: no longer used in v3.0 (kept for compilation compatibility)
      return metaPath.resolve("episodes_stats.jsonl");
   }

   public SideDependentList<Path> getZedVideoDirs()
   {
      return zedVideoDirs;
   }

   public float getFps()
   {
      return fps;
   }

   public List<String> getTaskNames()
   {
      return taskNames;
   }

   public List<LeRobotDatasetEpisode> getEpisodes()
   {
      return episodes;
   }

   public int getTotalEpisodeFrames()
   {
      int total = 0;
      for (LeRobotDatasetEpisode episode : episodes)
         total += episode.getLength();
      return total;
   }

   public long getTotalFrames()
   {
      return totalFrames;
   }

   public boolean isHasVideo()
   {
      return hasVideo;
   }
}
