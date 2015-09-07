package us.ihmc.simulationconstructionset.bambooTools.videoWall;

import java.io.File;
import java.io.FileFilter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import org.apache.commons.lang3.SystemUtils;

import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.io.printing.PrintTools;

public class BambooVideoWall
{
   public static void startVideoWallSourcedFromErasableDirectory()
   {
      startVideoWall(new LatestDirectoryProvider()
      {
         @Override
         public File checkForLatestDirectory()
         {
            return BambooTools.getEraseableDirectoryWithMostRecentBambooDataAndVideos();
         }
      });
   }

   public static void startVideoWallSourcedFromSamson()
   {
      startVideoWall(new LatestDirectoryProvider()
      {
         @Override
         public File checkForLatestDirectory()
         {
            if (SystemUtils.IS_OS_WINDOWS)
            {
               return BambooTools.getDirectoryWithMostRecentBambooDataAndVideos("//samson/BambooVideos");
            }
            else if (SystemUtils.IS_OS_MAC)
            {
               return BambooTools.getDirectoryWithMostRecentBambooDataAndVideos("/Users/unknownid/BambooVideos");
            }
            else
            {
               return null;
            }
         }
      });
   }
   
   private static void startVideoWall(LatestDirectoryProvider latestDirectoryProvider)
   {
      int numberOfRowsOfVideo = 2;
      int numberOfColumnsOfVideo = 3;
      MultipleVideoDecoderAndPlaybacker multipleVideoDecoderAndPlaybacker = new MultipleVideoDecoderAndPlaybacker(numberOfRowsOfVideo, numberOfColumnsOfVideo);

      while (true)
      {
         File latestDirectory = latestDirectoryProvider.checkForLatestDirectory();
         PrintTools.info("Latest directory = " + latestDirectory);
         showLatestMovies(latestDirectory, multipleVideoDecoderAndPlaybacker);

         ThreadTools.sleepSeconds(1.0);
      }
   }
   
   private interface LatestDirectoryProvider
   {
      File checkForLatestDirectory();
   }

   private static void showLatestMovies(File movieDirectory, MultipleVideoDecoderAndPlaybacker multipleVideoDecoderAndPlaybacker)
   {
      BambooTools.reportOutMessage("Showing Most Recent Movie in " + movieDirectory);

      FileFilter filter = new FileFilter()
      {
         public boolean accept(File file)
         {
            if (file.isDirectory())
               return false;
            if (file.getName().endsWith(".mpg"))
               return true;
            if (file.getName().endsWith(".mov"))
               return true;
            if (file.getName().endsWith(".mp4"))
               return true;

            return false;
         }
      };

      File[] potentialMovies = movieDirectory.listFiles(filter);

      if ((potentialMovies == null) || (potentialMovies.length == 0))
         return;

      ArrayList<File> moviesToPlay = new ArrayList<File>();

      Comparator<File> fileAlphabeticalComparator = BambooTools.createFileAlphabeticalComparator();
      Arrays.sort(potentialMovies, fileAlphabeticalComparator);

      int maxNumberOfMoviesToShow = multipleVideoDecoderAndPlaybacker.getTotalNumberOfScreens();

      ArrayList<String> videoNames = new ArrayList<String>();

      for (int i = 0; i < potentialMovies.length; i++)
      {
         if (moviesToPlay.size() < maxNumberOfMoviesToShow)
         {
            String videoName = potentialMovies[i].getName();

            String shortName = videoName.substring(14, videoName.length());
            PrintTools.info(shortName);

            if (!videoNames.contains(shortName))
            {
               moviesToPlay.add(potentialMovies[i]);

               videoNames.add(shortName);
            }

         }
      }

      for (int i = 0; i < moviesToPlay.size(); i++)
      {
         File movieToPlay = moviesToPlay.get(i);
         BambooTools.reportOutMessage("Playing " + movieToPlay);

         try
         {
            multipleVideoDecoderAndPlaybacker.playVideo(i, movieToPlay.getAbsolutePath());
         }
         catch (Exception e)
         {
            System.err.println("Could not play movie " + movieToPlay.getAbsolutePath());
         }

         BambooTools.reportOutMessage("Done Playing Movie " + movieToPlay);

      }

      while (!multipleVideoDecoderAndPlaybacker.areAllVideosDonePlaying())
      {
         ThreadTools.sleepSeconds(1.0);
      }
   }

   public static void main(String[] args)
   {
      // startVideoWallSourcedFromErasableDirectory();
      startVideoWallSourcedFromSamson();
   }
}
