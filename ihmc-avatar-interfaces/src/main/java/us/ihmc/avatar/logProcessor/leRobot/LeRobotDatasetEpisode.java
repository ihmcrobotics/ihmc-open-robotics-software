package us.ihmc.avatar.logProcessor.leRobot;

import com.fasterxml.jackson.databind.node.ArrayNode;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONFileTools;

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

   private int length = 0;

   public LeRobotDatasetEpisode(int episodeIndex, String episodeName, String taskName, Path episodesJsonlPath, Path episodeStatsJsonlPath, Path dataChunk0Path, SideDependentList<Path> zedVideoDirs)
   {
      this.episodeIndex = episodeIndex;
      this.episodeName = episodeName;
      this.taskName = taskName;
      this.episodesJsonlPath = episodesJsonlPath;
      this.episodeStatsJsonlPath = episodeStatsJsonlPath;
      this.dataChunk0Path = dataChunk0Path;
      this.zedVideoDirs = zedVideoDirs;
   }

   public void writeToFilesystem()
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
}
