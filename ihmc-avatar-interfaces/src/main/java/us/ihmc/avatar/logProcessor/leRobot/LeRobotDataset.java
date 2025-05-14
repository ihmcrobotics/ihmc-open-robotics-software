package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONFileTools;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class LeRobotDataset
{
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

   private final List<String> taskNames = new ArrayList<>();
   private final List<LeRobotDatasetEpisode> episodes = new ArrayList<>();
   long totalFrames = 0L;

   public LeRobotDataset(Path directory)
   {
      this.directory = directory;

      name = directory.getFileName().toString();
      dataPath = directory.resolve("data");
      metaPath = directory.resolve("meta");
      videosPath = directory.resolve("videos");
      dataChunk0Path = dataPath.resolve("chunk-000");
      for (RobotSide side : RobotSide.values)
         zedVideoDirs.put(side, videosPath.resolve("chunk-000/observations.images.cam_zed_" + side.getLowerCaseName()));

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
      taskNames.clear();
      JSONFileTools.loadLines(tasksJsonlPath, lineRoot ->
      {
         taskNames.add(lineRoot.get("task").textValue());
      });
      episodes.clear();
      JSONFileTools.loadLines(episodesJsonlPath, lineRoot ->
      {
         int episodeIndex = episodes.size();
         String taskName = lineRoot.get("tasks").get(0).textValue();
         int length = lineRoot.get("length").intValue();
         LeRobotDatasetEpisode episode = new LeRobotDatasetEpisode(episodeIndex,
                                                                   taskName,
                                                                   length,
                                                                   totalFrames,
                                                                   episodesJsonlPath,
                                                                   episodeStatsJsonlPath,
                                                                   dataChunk0Path,
                                                                   zedVideoDirs);
         episodes.add(episode);

         totalFrames += length;
      });
   }

   public void addEpisode(String taskName, SCS2LogSessionWithVideo session)
   {
      if (!taskNames.contains(taskName))
      {
         taskNames.add(taskName);
         writeTaskJsonlLine(taskName);
      }

      int episodeIndex = episodes.size();
      LeRobotDatasetEpisode episode = new LeRobotDatasetEpisode(episodeIndex,
                                                                taskName,
                                                                0,
                                                                totalFrames,
                                                                episodesJsonlPath,
                                                                episodeStatsJsonlPath,
                                                                dataChunk0Path,
                                                                zedVideoDirs);
      episode.startGeneratingEpisode(session, this::writeMetaJson);
      episodes.add(episode);
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

   public void writeMetaJson()
   {
      JSONFileTools.save(infoJsonPath, rootNode ->
      {
         totalFrames = 0;
         float fps = episodes.isEmpty() ? 1.0f : episodes.get(0).getFps();
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
            ObjectNode cam = features.putObject("observations.images.cam_zed_%s".formatted(side.getLowerCaseName()));
            cam.put("dtype", "video");
            cam.putArray("shape").add(480).add(640).add(3);
            cam.putArray("names").add("height").add("width").add("channel");
            cam.putObject("video_info").put("video.fps", fps)
                                       .put("video.codec", "mpeg4")
                                       .put("video.pix_fmt", "yuv420p")
                                       .put("video.is_depth_map", false)
                                       .put("has_audio", false);
         }

         ObjectNode state = features.putObject("state");
         state.put("dtype", "float32");
         state.putArray("shape").add(14);
         ArrayNode motors = state.putObject("names").putArray("motors");
         motors.add("left_gripper_x").add("left_gripper_y").add("left_gripper_z");
         motors.add("left_gripper_qx").add("left_gripper_qy").add("left_gripper_qz").add("left_gripper_qs");
         motors.add("right_gripper_x").add("right_gripper_y").add("right_gripper_z");
         motors.add("right_gripper_qx").add("right_gripper_qy").add("right_gripper_qz").add("right_gripper_qs");
         ObjectNode action = features.putObject("action");
         action.put("dtype", "float32");
         action.putArray("shape").add(14);
         motors = action.putObject("names").putArray("motors");
         motors.add("left_gripper_x").add("left_gripper_y").add("left_gripper_z");
         motors.add("left_gripper_qx").add("left_gripper_qy").add("left_gripper_qz").add("left_gripper_qs");
         motors.add("right_gripper_x").add("right_gripper_y").add("right_gripper_z");
         motors.add("right_gripper_qx").add("right_gripper_qy").add("right_gripper_qz").add("right_gripper_qs");

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

   public SideDependentList<Path> getZedVideoDirs()
   {
      return zedVideoDirs;
   }

   public List<String> getTaskNames()
   {
      return taskNames;
   }

   public List<LeRobotDatasetEpisode> getEpisodes()
   {
      return episodes;
   }

   public long getTotalFrames()
   {
      return totalFrames;
   }
}
