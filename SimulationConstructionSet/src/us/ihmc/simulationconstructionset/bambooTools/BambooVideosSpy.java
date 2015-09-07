package us.ihmc.simulationconstructionset.bambooTools;

import java.io.File;
import java.io.FileFilter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import org.apache.commons.lang3.SystemUtils;

import us.ihmc.utilities.io.printing.PrintTools;

public class BambooVideosSpy
{
   public static void spyOnEraseableBambooDataAndVideosDirectory()
   {
      spyOnLatestBambooDataAndVideos(new LatestDirectoryProvider()
      {
         @Override
         public File findLatestDirectory()
         {
            return BambooTools.getEraseableDirectoryWithMostRecentBambooDataAndVideos();
         }
      });
   }

   public static void spyOnLatestBambooDataAndVideosOnSamson()
   {
      spyOnLatestBambooDataAndVideos(new LatestDirectoryProvider()
      {
         @Override
         public File findLatestDirectory()
         {
            if (SystemUtils.IS_OS_WINDOWS)
               return BambooTools.getDirectoryWithMostRecentBambooDataAndVideos("//samson/BambooVideos");
            else if (SystemUtils.IS_OS_MAC)
               return BambooTools.getDirectoryWithMostRecentBambooDataAndVideos("/Users/unknownid/BambooVideos");
            else
               return null;
         }
      });
   }
   
   private static void spyOnLatestBambooDataAndVideos(LatestDirectoryProvider latestDirectoryProvider)
   {
      int numberOfRowsOfVideo = 2;
      int numberOfColumnsOfVideo = 3;
      MultipleVideoDecoderAndPlaybacker multipleVideoDecoderAndPlaybacker = new MultipleVideoDecoderAndPlaybacker(numberOfRowsOfVideo, numberOfColumnsOfVideo);

      while (true)
      {
         File mostRecentDirectory = latestDirectoryProvider.findLatestDirectory();
         
         PrintTools.info("Most recent directory = " + mostRecentDirectory);

         showMostRecentMovies(mostRecentDirectory, multipleVideoDecoderAndPlaybacker);

         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException e)
         {
         }
      }
   }
   
   private interface LatestDirectoryProvider
   {
      File findLatestDirectory();
   }

   private static void showMostRecentMovies(File movieDirectory, MultipleVideoDecoderAndPlaybacker multipleVideoDecoderAndPlaybacker)
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
            System.out.println("BambooVideoSpy:" + shortName);

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
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException e)
         {
         }

      }
   }

   public static void main(String[] args)
   {
      // spyOnEraseableBambooDataAndVideosDirectory();
      spyOnLatestBambooDataAndVideosOnSamson();
   }
}
