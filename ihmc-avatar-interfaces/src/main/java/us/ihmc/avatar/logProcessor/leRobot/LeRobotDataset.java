package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.yoVariables.variable.YoInteger;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Represents a LeRobot dataset (a huggingface format) in our system for generating datasets from IHMC logs.
 */
public class LeRobotDataset
{
   public static boolean XYZ_RIGHT_ONLY = Boolean.parseBoolean(System.getProperty("xyz.right.only", "true"));
   public static int STATE_SIZE = XYZ_RIGHT_ONLY ? 3 : 28;

   private final HumanoidJointNameMap jointMap;
   private final HumanoidRobotSensorInformation sensorInformation;
   private final String name;
   private final Path directory;
   private final Path dataPath;
   private final Path metaPath;
   private final Path videosPath;
   private final Path dataChunk0Path;
   private final SideDependentList<Path> zedVideoDirs = new SideDependentList<>();
   private final Path episodesJsonlPath;
   private final Path episodeStatsJsonlPath;
   private final Path infoJsonPath;
   private final Path tasksJsonlPath;

   private float fps = 30.0f;
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
      for (RobotSide side : RobotSide.values)
         zedVideoDirs.put(side, videosPath.resolve("chunk-000/observation.images.cam_zed_" + side.getLowerCaseName()));

      episodesJsonlPath = metaPath.resolve("episodes.jsonl");
      episodeStatsJsonlPath = metaPath.resolve("episodes_stats.jsonl");
      infoJsonPath = metaPath.resolve("info.json");
      tasksJsonlPath = metaPath.resolve("tasks.jsonl");
   }

   public void mkdirs()
   {
      FileTools.ensureDirectoryExists(directory, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(dataPath, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(metaPath, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(videosPath, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(dataChunk0Path, DefaultExceptionHandler.PRINT_MESSAGE);
      for (RobotSide side : RobotSide.values)
         FileTools.ensureDirectoryExists(zedVideoDirs.get(side), DefaultExceptionHandler.PRINT_MESSAGE);

      FileTools.ensureFileExists(episodesJsonlPath, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureFileExists(episodeStatsJsonlPath, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureFileExists(infoJsonPath, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureFileExists(tasksJsonlPath, DefaultExceptionHandler.PRINT_MESSAGE);
   }

   public void loadData() // TODO: Load completely
   {
      JSONFileTools.load(infoJsonPath, rootNode -> {
         fps = rootNode.get("fps").floatValue();
      });
      taskNames.clear();
      JSONFileTools.loadLines(tasksJsonlPath, lineRoot ->
      {
         taskNames.add(lineRoot.get("task").textValue());
      });
      episodes.clear();
      JSONFileTools.loadLines(episodesJsonlPath, lineRoot ->
      {
         LeRobotDatasetEpisode episode = new LeRobotDatasetEpisode(this, lineRoot, jointMap, sensorInformation);
         episode.loadParquetData();
         episodes.add(episode);
         totalFrames += episode.getLength();
      });
      MutableInt i = new MutableInt();
      JSONFileTools.loadLines(episodeStatsJsonlPath, lineRoot ->
      {
         episodes.get(i.intValue()).getStatistics().loadJSON(lineRoot);
         i.increment();
      });
   }

   public void addEpisode(String taskName, SCS2LogSessionWithVideo session)
   {
      createEpisode(taskName).generateFromActiveBuffer(session);
   }

   private LeRobotDatasetEpisode createEpisode(String taskName)
   {
      ensureTaskNameInJsonl(taskName);
      LeRobotDatasetEpisode episode = new LeRobotDatasetEpisode(this, episodes.size(), taskName, jointMap, sensorInformation);
      episodes.add(episode);
      return episode;
   }

   public BooleanSupplier addEpisodesAutomatically(String taskName, int taskID, SCS2LogSessionWithVideo session, BooleanSupplier keepGoing)
   {
      ensureTaskNameInJsonl(taskName);

      MutableBoolean stillGoing = new MutableBoolean(false);
      String kstModule = LeRobotDatasetTools.findRegistry(session.getRootRegistry(), "root.main", "IKStreamingRTThread");
      String kstStreaming = kstModule + "KinematicsStreamingToolboxController.KSTStreamingState.";
      if (session.getRootRegistry().findVariable(kstStreaming + "demonstrationTaskID") instanceof YoInteger demonstrationTaskID)
      {
         ThreadTools.startAThread(() ->
         {
            try
            {
               LeRobotDatasetEpisode episode = null;
               int desiredLoadedIndex = Math.max(0, session.getLogDataReader().getCurrentLogPosition() - 1);
               while (keepGoing.getAsBoolean() && desiredLoadedIndex > -1)
               {
                  // The current log position is actually referring to the next log index to read
                  // so when currentLogPosition is 7, it mean we have just read position 6 into the buffer
                  int indexToLoad = session.getLogDataReader().getCurrentLogPosition();
                  int loadedIndex = indexToLoad - 1;
                  boolean desiredDataIsLoaded = loadedIndex == desiredLoadedIndex;
                  if (desiredDataIsLoaded)
                  {
                     if (demonstrationTaskID.getValue() == taskID)
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
                        episode.finalizeEpisodeGeneration();
                        episode = null;
                     }
                  }

                  if (indexToLoad < session.getLogDataReader().getNumberOfEntries())
                  {
                     session.submitLogPositionRequest(indexToLoad);
                     desiredLoadedIndex = indexToLoad;
                  }
                  else // we hit the end
                  {
                     break;
                  }
                  ThreadTools.park(0.000001);
               }
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
      return stillGoing::booleanValue;
   }

   private void ensureTaskNameInJsonl(String taskName)
   {
      if (!taskNames.contains(taskName))
      {
         taskNames.add(taskName);
         writeTaskJsonlLine(taskName);
      }
   }

   public void removeEpisodes(boolean[] episodesToRemove)
   {
      ExceptionTools.handle(() ->
      {
         List<String> statsLines = Files.readAllLines(episodeStatsJsonlPath);

         for (int i = 0, j = 0; i < episodesToRemove.length; i++, j++)
         {
            if (episodesToRemove[i])
            {
               FileTools.deleteQuietly(dataChunk0Path.resolve("episode_%06d".formatted(j) + ".parquet"));
               for (RobotSide side : RobotSide.values)
                  FileTools.deleteQuietly(zedVideoDirs.get(side).resolve("episode_%06d".formatted(j) + ".mp4"));

               episodes.remove(j);
               statsLines.remove(j);

               for (int k = j; k < episodes.size(); k++)
               {
                  episodes.get(k).reindex(k);

                  Files.move(dataChunk0Path.resolve("episode_%06d".formatted(k + 1) + ".parquet"),
                             dataChunk0Path.resolve("episode_%06d".formatted(k) + ".parquet"));
                  for (RobotSide side : RobotSide.values)
                     Files.move(zedVideoDirs.get(side).resolve("episode_%06d".formatted(k + 1) + ".mp4"),
                                zedVideoDirs.get(side).resolve("episode_%06d".formatted(k) + ".mp4"));

                  final String statsLine = statsLines.get(k);
                  ObjectNode node = (ObjectNode) ExceptionTools.handle(() -> new ObjectMapper(new JsonFactory()).readTree(statsLine),
                                                                       DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
                  node.put("episode_index", k);
                  ObjectNode episodeIndex = (ObjectNode) node.get("stats").get("episode_index");
                  episodeIndex.putArray("min").add(k);
                  episodeIndex.putArray("max").add(k);
                  episodeIndex.putArray("mean").add((double) k);
                  LeRobotIntegerStatisticsCalculator indexStats = new LeRobotIntegerStatisticsCalculator();
                  int priorFrames = 0;
                  for (int l = 0; l < k; l++)
                     priorFrames += episodes.get(l).getLength();
                  for (int l = priorFrames; l < priorFrames + episodes.get(k).getLength(); l++)
                     indexStats.addValue(l);
                  indexStats.calculate();
                  ObjectNode index = (ObjectNode) node.get("stats").get("index");
                  index.putArray("min").add(indexStats.getMin());
                  index.putArray("max").add(indexStats.getMax());
                  index.putArray("mean").add(indexStats.getMean());
                  index.putArray("std").add(indexStats.getStddev());

                  statsLines.set(k, node.toString());
               }

               --j;
            }
         }

         for (LeRobotDatasetEpisode episode : episodes)
            episode.writeParquetData();

         FileTools.write(episodesJsonlPath, new byte[0], WriteOption.TRUNCATE, DefaultExceptionHandler.PRINT_MESSAGE);
         for (LeRobotDatasetEpisode episode : episodes)
            episode.writeEpisodeJsonlLine();

         Files.write(episodeStatsJsonlPath, statsLines, StandardOpenOption.TRUNCATE_EXISTING);

         writeMetaJson();

      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   private void shiftEpisodeIndicesInJsonl(Path jsonlPath, int removedIndex) throws IOException
   {
      List<String> allLines = Files.readAllLines(jsonlPath);

      ObjectMapper mapper = new ObjectMapper();
      List<String> rewritten = new ArrayList<>(allLines.size());

      for (int lineIdx = 0; lineIdx < allLines.size(); lineIdx++)
      {
         String line = allLines.get(lineIdx).trim();
         if (line.isEmpty())
         {
            rewritten.add(line);
            continue;
         }

         JsonNode root = mapper.readTree(line);

         JsonNode episodeIndexNode = root.get("episode_index");
         if (episodeIndexNode != null && episodeIndexNode.isInt())
         {
            int oldIndex = episodeIndexNode.intValue();
            if (oldIndex > removedIndex)
            {
               ((ObjectNode) root).put("episode_index", oldIndex - 1);
            }
         }
         rewritten.add(mapper.writeValueAsString(root));
      }
      Files.write(jsonlPath, rewritten, StandardOpenOption.TRUNCATE_EXISTING);
      LogTools.info("Shifted episode_index in JSONL: " + jsonlPath + " (removedIndex=" + removedIndex + ").");
   }

   private void removeLineFromJsonl(Path jsonlPath, int index) throws IOException
   {
      List<String> allLines = Files.readAllLines(jsonlPath);
      List<String> linesToWrite = new ArrayList<>();
      List<String> secondLines = new ArrayList<>();
      if (index > 0)
         linesToWrite = allLines.subList(0, index - 1);
      if (index < allLines.size())
         secondLines = allLines.subList(index + 1, allLines.size());
      linesToWrite.addAll(secondLines);
      Files.write(jsonlPath, linesToWrite, StandardOpenOption.TRUNCATE_EXISTING);
      LogTools.info("Removed last line from " + jsonlPath + " (now has " + linesToWrite.size() + " lines).");
   }

   public void regenerateAndRewriteMetadata()
   {
      writeMetaJson();

      FileTools.write(episodesJsonlPath, new byte[0], WriteOption.TRUNCATE, DefaultExceptionHandler.PRINT_MESSAGE);
      for (LeRobotDatasetEpisode episode : episodes)
      {
         episode.writeEpisodeJsonlLine();
      }

      FileTools.write(episodeStatsJsonlPath, new byte[0], WriteOption.TRUNCATE, DefaultExceptionHandler.PRINT_MESSAGE);
      for (LeRobotDatasetEpisode episode : episodes)
      {
         LogTools.info("Generating stats for %s...".formatted(episode.getEpisodeName()));
         episode.readDataAndWriteStatisticsJsonlLine();
      }

      FileTools.write(tasksJsonlPath, new byte[0], WriteOption.TRUNCATE, DefaultExceptionHandler.PRINT_MESSAGE);
      for (String taskName : taskNames)
      {
         writeTaskJsonlLine(taskName);
      }
      LogTools.info("All done regenerating and rewriting metadata.");
   }

   public void writeParquetData()
   {
      for (LeRobotDatasetEpisode episode : episodes)
      {
         episode.writeParquetData();
      }
   }

   public void writeMetaJson()
   {
      JSONFileTools.save(infoJsonPath, rootNode ->
      {
         totalFrames = 0;
         for (LeRobotDatasetEpisode episode : episodes)
         {
            totalFrames += episode.getLength();
         }

         rootNode.put("codebase_version", "v2.1");
         rootNode.put("robot_type", "nadia");
         rootNode.put("total_episodes", episodes.size());
         rootNode.put("total_frames", totalFrames);
         rootNode.put("total_tasks", taskNames.size());
         rootNode.put("total_videos", 2 * episodes.size());
         rootNode.put("total_chunks", 1);
         rootNode.put("chunks_size", 1000);
         rootNode.put("fps", fps);
         ObjectNode splits = rootNode.putObject("splits");
         splits.put("train", "0:%d".formatted(episodes.size()));
         rootNode.put("data_path", "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet");
         rootNode.put("video_path", "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4");

         ObjectNode features = rootNode.putObject("features");
         for (RobotSide side : RobotSide.values)
         {
            ObjectNode cam = features.putObject("observation.images.cam_zed_%s".formatted(side.getLowerCaseName()));
            cam.put("dtype", "video");
            cam.putArray("shape").add(480).add(640).add(3);
            cam.putArray("names").add("height").add("width").add("channel");
            cam.putObject("video_info").put("video.fps", fps)
                                       .put("video.codec", "mpeg4")
                                       .put("video.pix_fmt", "yuv420p")
                                       .put("video.is_depth_map", false)
                                       .put("has_audio", false);
         }

         if (!episodes.isEmpty() && !episodes.get(0).getRecords().isEmpty())
         {
            LeRobotEpisodeRecord record = episodes.get(0).getRecords().get(0);
            int shape = record.state().size();
            ObjectNode state = features.putObject("observation.state");
            state.put("dtype", "float32");
            state.putArray("shape").add(shape);
            ArrayNode motors = state.putObject("names").putArray("motors");
            if (shape == 3)
            {
               motors.add("right_gripper_x").add("right_gripper_y").add("right_gripper_z");
            }
            else
            {
               motors.add("left_gripper_x").add("left_gripper_y").add("left_gripper_z");
               motors.add("left_gripper_qx").add("left_gripper_qy").add("left_gripper_qz").add("left_gripper_qs");
               motors.add("left_forearm_x").add("left_forearm_y").add("left_forearm_z");
               motors.add("left_forearm_qx").add("left_forearm_qy").add("left_forearm_qz").add("left_forearm_qs");
               motors.add("right_gripper_x").add("right_gripper_y").add("right_gripper_z");
               motors.add("right_gripper_qx").add("right_gripper_qy").add("right_gripper_qz").add("right_gripper_qs");
               motors.add("right_forearm_x").add("right_forearm_y").add("right_forearm_z");
               motors.add("right_forearm_qx").add("right_forearm_qy").add("right_forearm_qz").add("right_forearm_qs");
            }
            ObjectNode action = features.putObject("action");
            action.put("dtype", "float32");
            action.putArray("shape").add(shape);
            motors = action.putObject("names").putArray("motors");
            if (shape == 3)
            {
               motors.add("right_gripper_x").add("right_gripper_y").add("right_gripper_z");
            }
            else
            {
               motors.add("left_gripper_x").add("left_gripper_y").add("left_gripper_z");
               motors.add("left_gripper_qx").add("left_gripper_qy").add("left_gripper_qz").add("left_gripper_qs");
               motors.add("left_forearm_x").add("left_forearm_y").add("left_forearm_z");
               motors.add("left_forearm_qx").add("left_forearm_qy").add("left_forearm_qz").add("left_forearm_qs");
               motors.add("right_gripper_x").add("right_gripper_y").add("right_gripper_z");
               motors.add("right_gripper_qx").add("right_gripper_qy").add("right_gripper_qz").add("right_gripper_qs");
               motors.add("right_forearm_x").add("right_forearm_y").add("right_forearm_z");
               motors.add("right_forearm_qx").add("right_forearm_qy").add("right_forearm_qz").add("right_forearm_qs");
            }
         }

         ObjectNode episodeIndex = features.putObject("episode_index");
         episodeIndex.put("dtype", "int64");
         episodeIndex.putArray("shape").add(1);
         episodeIndex.put("names", (byte[]) null);
         ObjectNode frameIndex = features.putObject("frame_index");
         frameIndex.put("dtype", "int64");
         frameIndex.putArray("shape").add(1);
         frameIndex.put("names", (byte[]) null);
         ObjectNode timestamp = features.putObject("timestamp");
         timestamp.put("dtype", "float32");
         timestamp.putArray("shape").add(1);
         timestamp.put("names", (byte[]) null);
         ObjectNode nextDone = features.putObject("next.done");
         nextDone.put("dtype", "bool");
         nextDone.putArray("shape").add(1);
         nextDone.put("names", (byte[]) null);
         ObjectNode index = features.putObject("index");
         index.put("dtype", "int64");
         index.putArray("shape").add(1);
         index.put("names", (byte[]) null);
         ObjectNode taskIndex = features.putObject("task_index");
         taskIndex.put("dtype", "int64");
         taskIndex.putArray("shape").add(1);
         taskIndex.put("names", (byte[]) null);
      });
   }

   private void writeTaskJsonlLine(String taskName)
   {
      LeRobotDatasetTools.appendLine(tasksJsonlPath, JSONFileTools.getAsSingleLine(node ->
      {
         node.put("task_index", taskNames.size() - 1);
         node.put("task", taskName);
      }));
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
      return episodesJsonlPath;
   }

   public Path getEpisodeStatsJsonlPath()
   {
      return episodeStatsJsonlPath;
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
      int totalFrames = 0;
      for (LeRobotDatasetEpisode episode : episodes)
         totalFrames += episode.getLength();
      return totalFrames;
   }

   public long getTotalFrames()
   {
      return totalFrames;
   }
}
