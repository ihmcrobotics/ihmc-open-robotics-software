package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONFileTools;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class LeRobotDataset
{
   public static final double ZED_FPS = 15.0;

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
      episodeStatsJsonlPath = metaPath.resolve("episode_stats.jsonl");
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

   public void loadData()
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
         LeRobotDatasetEpisode episode = new LeRobotDatasetEpisode(episodeIndex,
                                                                   taskName,
                                                                   episodesJsonlPath,
                                                                   episodeStatsJsonlPath,
                                                                   dataChunk0Path,
                                                                   zedVideoDirs);
         episodes.add(episode);
      });
   }

   public void addEpisode(String taskName, SCS2LogSessionWithVideo session)
   {
      if (!taskNames.contains(taskName))
      {
         taskNames.add(taskName);

         LeRobotDatasetTools.appendLine(tasksJsonlPath, JSONFileTools.getAsSingleLine(node ->
         {
            node.put("task_index", taskNames.size() - 1);
            node.put("task", taskName);
         }));
      }

      // add episode to episodes.jsonl

      int episodeIndex = episodes.size();
      LeRobotDatasetEpisode episode = new LeRobotDatasetEpisode(episodeIndex,
                                                                taskName,
                                                                episodesJsonlPath,
                                                                episodeStatsJsonlPath,
                                                                dataChunk0Path,
                                                                zedVideoDirs);
      episode.startGeneratingEpisode(session);
      episodes.add(episode);

      // add episode stats entry
      // update info.json

      // add episode parquet file
      // add mp4 for each camera
   }

   public void writeMetadataToFilesystem()
   {
      JSONFileTools.save(infoJsonPath, rootNode ->
      {
         int totalFrames = 0;
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
         rootNode.put("chunk_size", 1000);
         rootNode.put("fps", ZED_FPS);
         ObjectNode splits = rootNode.putObject("splits");
         splits.put("train", "0:%d".formatted(episodes.size()));
         rootNode.put("data_path", "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet");
         rootNode.put("video_path", "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4");

         ObjectNode features = rootNode.putObject("features");
         for (RobotSide side : RobotSide.values)
         {

         }
      });
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
}
