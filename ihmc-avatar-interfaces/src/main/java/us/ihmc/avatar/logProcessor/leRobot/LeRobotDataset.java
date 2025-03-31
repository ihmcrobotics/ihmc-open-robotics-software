package us.ihmc.avatar.logProcessor.leRobot;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONFileTools;

import java.nio.file.Path;
import java.nio.file.Files;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;

public class LeRobotDataset
{
   private final String name;
   private final Path directory;
   private final Path data;
   private final Path meta;
   private final Path videos;
   private final Path dataChunk0;
   private final SideDependentList<Path> zedVideoDirs = new SideDependentList<>();
   private final Path episodes;
   private final Path episodeStats;
   private final Path info;
   private final Path tasks;

   private final List<String> taskNames = new ArrayList<>();

   public LeRobotDataset(Path directory)
   {
      this.directory = directory;

      name = directory.getFileName().toString();
      data = directory.resolve("data");
      meta = directory.resolve("meta");
      videos = directory.resolve("videos");
      dataChunk0 = data.resolve("chunk-000");
      for (RobotSide side : RobotSide.values)
         zedVideoDirs.put(side, videos.resolve("chunk-000/observations.images.cam_zed_" + side.getLowerCaseName()));

      episodes = meta.resolve("episodes.jsonl");
      episodeStats = meta.resolve("episode_stats.jsonl");
      info = meta.resolve("info.json");
      tasks = meta.resolve("tasks.jsonl");
   }

   public void mkdirs()
   {
      FileTools.ensureDirectoryExists(directory, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(data, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(meta, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(videos, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureDirectoryExists(dataChunk0, DefaultExceptionHandler.PRINT_MESSAGE);
      for (RobotSide side : RobotSide.values)
         FileTools.ensureDirectoryExists(zedVideoDirs.get(side), DefaultExceptionHandler.PRINT_MESSAGE);

      FileTools.ensureFileExists(episodes, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureFileExists(episodeStats, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureFileExists(info, DefaultExceptionHandler.PRINT_MESSAGE);
      FileTools.ensureFileExists(tasks, DefaultExceptionHandler.PRINT_MESSAGE);
   }

   public void loadData()
   {
      taskNames.clear();
      JSONFileTools.loadLines(tasks, lineRoot ->
      {
         taskNames.add(lineRoot.get("task").textValue());
      });
   }

   public void addEpisode(String taskName)
   {
      if (!taskNames.contains(taskName))
      {
         taskNames.add(taskName);

         appendLine(tasks, JSONFileTools.getAsSingleLine(node ->
         {
            node.put("task_index", taskNames.size() - 1);
            node.put("task", taskName);
         }));
      }

      // add episode to episodes.jsonl
      // add episode stats entry
      // update info.json

      // add episode parquet file
      // add mp4 for each camera
   }

   private void appendLine(Path path, String line)
   {
      ExceptionTools.handle(() -> Files.writeString(path, line, StandardOpenOption.APPEND), DefaultExceptionHandler.PRINT_MESSAGE);
   }

   public String getName()
   {
      return name;
   }

   public Path getDirectory()
   {
      return directory;
   }

   public Path getData()
   {
      return data;
   }

   public Path getMeta()
   {
      return meta;
   }

   public Path getVideos()
   {
      return videos;
   }

   public Path getDataChunk0()
   {
      return dataChunk0;
   }

   public SideDependentList<Path> getZedVideoDirs()
   {
      return zedVideoDirs;
   }

   public List<String> getTaskNames()
   {
      return taskNames;
   }
}
