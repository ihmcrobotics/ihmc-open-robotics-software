package us.ihmc.avatar.logProcessor.leRobot;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.nio.file.Path;

public class LeRobotDataset
{
   private final Path directory;
   private final Path data;
   private final Path meta;
   private final Path videos;
   private final Path dataChunk0;
   private final SideDependentList<Path> zedVideoDirs = new SideDependentList<>();

   public LeRobotDataset(Path directory)
   {
      this.directory = directory;
      data = directory.resolve("data");
      meta = directory.resolve("meta");
      videos = directory.resolve("videos");
      dataChunk0 = data.resolve("chunk-000");
      for (RobotSide side : RobotSide.values)
         zedVideoDirs.put(side, videos.resolve("chunk-000/observations.images.cam_zed_" + side.getLowerCaseName()));
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

      FileTools.ensureFileExists(meta.resolve("info.json"), DefaultExceptionHandler.PRINT_MESSAGE);
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
}
