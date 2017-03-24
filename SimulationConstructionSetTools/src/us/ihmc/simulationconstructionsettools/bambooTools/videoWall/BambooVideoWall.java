package us.ihmc.simulationconstructionsettools.bambooTools.videoWall;

import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.swing.ImageIcon;
import javax.swing.JFrame;

import org.apache.commons.lang3.SystemUtils;

import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.tools.thread.ThreadTools;

public class BambooVideoWall
{
   private static final double PLAYBACKER_HEIGHT = 1100;
   private static final int NUMBER_OF_ROWS_OF_VIDEO = 5;
   private static final int NUMBER_OF_COLUMNS_OF_VIDEO = 5;
   private static final int NUMBER_OF_SCREENS = NUMBER_OF_ROWS_OF_VIDEO * NUMBER_OF_COLUMNS_OF_VIDEO;
   private static final int VIDEO_WIDTH = 1280;
   private static final int VIDEO_HEIGHT = 720;

   private JFrame jFrame;
   private VideoPlayerGrid videoPlayerGrid;

   public BambooVideoWall()
   {
      jFrame = new JFrame(BambooVideoWall.class.getSimpleName());

      double adjustedVideoHeight = PLAYBACKER_HEIGHT / (double) NUMBER_OF_ROWS_OF_VIDEO;
      double adjustedVideoWidth = adjustedVideoHeight * VIDEO_WIDTH / VIDEO_HEIGHT;
      double width = adjustedVideoWidth * NUMBER_OF_COLUMNS_OF_VIDEO;
      
      videoPlayerGrid = new VideoPlayerGrid(NUMBER_OF_ROWS_OF_VIDEO, NUMBER_OF_COLUMNS_OF_VIDEO);
      jFrame.getContentPane().add(videoPlayerGrid);

      jFrame.setSize((int) width, (int) PLAYBACKER_HEIGHT + jFrame.getInsets().top);
      jFrame.setIconImage(new ImageIcon(ClassLoader.getSystemResource("us/ihmc/tools/icons/running-man-32x32.png").getPath()).getImage());
      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      jFrame.setVisible(true);
   }

   public void startVideoWallSourcedFromErasableDirectory()
   {
      startVideoWall(new LatestDirectoryProvider()
      {
         @Override
         public Path checkForLatestDirectory()
         {
            return BambooTools.getEraseableDirectoryWithMostRecentBambooDataAndVideos();
         }
      });
   }

   public void startVideoWallSourcedFromSamson()
   {
      startVideoWall(new LatestDirectoryProvider()
      {
         @Override
         public Path checkForLatestDirectory()
         {
            if (SystemUtils.IS_OS_WINDOWS)
            {
               return BambooTools.getDirectoryWithMostRecentBambooDataAndVideos(Paths.get("//samson/BambooVideos"));
            }
            else if (SystemUtils.IS_OS_MAC)
            {
               return BambooTools.getDirectoryWithMostRecentBambooDataAndVideos(Paths.get("/Users/unknownid/BambooVideos"));
            }
            else
            {
               return null;
            }
         }
      });
   }

   private void startVideoWall(LatestDirectoryProvider latestDirectoryProvider)
   {
      Path latestDirectory = null;
      List<Path> videoList = null;

//      while (true)
//      {
         Path checkDirectory = latestDirectoryProvider.checkForLatestDirectory();

         if (latestDirectory == null || !latestDirectory.equals(checkDirectory))
         {
            latestDirectory = checkDirectory;
            videoList = getVideoPathListFromDirectory(latestDirectory);
         }
         
         showVideosOnce(videoList);

         ThreadTools.sleep(200);
//      }
   }

   private interface LatestDirectoryProvider
   {
      Path checkForLatestDirectory();
   }

   private List<Path> getVideoPathListFromDirectory(Path videoDirectory)
   {
      final List<Path> videoPaths = new ArrayList<>();

      PathTools.walkFlat(videoDirectory, new BasicPathVisitor()
      {
         @Override
         public FileVisitResult visitPath(Path path, PathType pathType)
         {
            if (pathType == PathType.FILE)
            {
               if (PathTools.getExtension(path).matches("mpg|mov|mp4"))
               {
                  videoPaths.add(path);
               }
            }

            return FileVisitResult.CONTINUE;
         }
      });

      Collections.sort(videoPaths);

      return videoPaths;
   }

   private void showVideosOnce(List<Path> videoPaths)
   {
      for (int videoIndex = 0, playerSlotIndex = 0; videoIndex < videoPaths.size(); videoIndex++, playerSlotIndex++)
      {
         if (playerSlotIndex == NUMBER_OF_SCREENS)
         {
            playerSlotIndex = 0;
         }

         while (!videoPlayerGrid.getVideoPlayers().get(playerSlotIndex).readyToUpdateVideoPath())
         {
            ThreadTools.sleep(200);
         }

         videoPlayerGrid.updateVideoPath(playerSlotIndex, videoPaths.get(videoIndex));
      }
   }

   public static void main(String[] args)
   {
      new BambooVideoWall().startVideoWallSourcedFromSamson();
   }
}
