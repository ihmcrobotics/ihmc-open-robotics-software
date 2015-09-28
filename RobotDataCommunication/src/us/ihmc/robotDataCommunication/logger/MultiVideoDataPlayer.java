package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import us.ihmc.robotDataCommunication.logger.util.CustomProgressMonitor;
import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.simulationconstructionset.PlaybackListener;

public class MultiVideoDataPlayer implements PlaybackListener, RewoundListener
{
   private final LongYoVariable timestamp;
   private final LogProperties logProperties;

   
   private final ArrayList<String> videos = new ArrayList<>();

   
   private final HashMap<String, VideoDataPlayer> players = new HashMap<>();

   private VideoDataPlayer activePlayer = null;

   public MultiVideoDataPlayer(File dataDirectory, LogProperties logProperties, LongYoVariable timestamp)
   {
      this.timestamp = timestamp;
      this.logProperties = logProperties;

      for (String video : logProperties.getVideoFiles())
      {
         try
         {
            VideoDataPlayer player = new VideoDataPlayer(video, dataDirectory, logProperties);
            players.put(video, player);
            videos.add(video);
         }
         catch (IOException e)
         {
            System.err.println(e.getMessage());
         }
      }

      if (players.size() > 0)
      {
         setActivePlayer(videos.get(0));
      }

   }

   public List<String> getVideos()
   {
      return Collections.unmodifiableList(videos);
   }

   @Override
   public void indexChanged(int newIndex, double newTime)
   {
      if (activePlayer != null)
      {
         activePlayer.showVideoFrame(timestamp.getLongValue());
      }
   }

   @Override
   public void wasRewound()
   {
      if (activePlayer != null)
      {
         activePlayer.showVideoFrame(timestamp.getLongValue());
      }
   }

   public void setActivePlayer(String name)
   {
      if (activePlayer != null)
      {
         activePlayer.setVisible(false);
      }

      if (name != null)
      {
         activePlayer = players.get(name);
         activePlayer.showVideoFrame(timestamp.getLongValue());
         activePlayer.setVisible(true);
      }
      else
      {
         activePlayer = null;
      }
   }

   @Override
   public void play(double realTimeRate)
   {

   }

   @Override
   public void stop()
   {

   }

   public long getCurrentTimestamp()
   {
      return timestamp.getLongValue();
   }

   public void exportCurrentVideo(File selectedFile, long startTimestamp, long endTimestamp)
   {
      if (activePlayer != null)
      {
         CustomProgressMonitor monitor = new CustomProgressMonitor("Exporting " + activePlayer.getName(), selectedFile.getAbsolutePath(), 0, 100);
         PrintStream output = new PrintStream(monitor.getOutputStream(), true);
         monitor.setProgress(10);
         activePlayer.exportVideo(selectedFile, startTimestamp, endTimestamp, output);
         monitor.close();
      }
   }

   public void crop(File selectedDirectory, long startTimestamp, long endTimestamp, CustomProgressMonitor monitor) throws IOException
   {
      PrintStream output = new PrintStream(monitor.getOutputStream(), true);

      for (int i = 0; i < videos.size(); i++)
      {
         String video = videos.get(i);
         if (monitor != null)
         {
            monitor.setNote("Cropping video " + video);
            monitor.setProgress(50 + ((int) (50.0 * ((double) i / (double) videos.size()))));
         }

         File timestampFile = new File(selectedDirectory, logProperties.getTimestampFile(video));
         File videoFile = new File(selectedDirectory, logProperties.getVideoFile(video));
         players.get(video).cropVideo(videoFile, timestampFile, startTimestamp, endTimestamp, output);
      }
   }

}
