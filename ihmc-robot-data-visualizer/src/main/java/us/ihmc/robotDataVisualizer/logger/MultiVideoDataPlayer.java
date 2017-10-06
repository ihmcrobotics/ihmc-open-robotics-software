package us.ihmc.robotDataVisualizer.logger;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import us.ihmc.robotDataLogger.Camera;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataVisualizer.logger.util.CustomProgressMonitor;
import us.ihmc.robotDataVisualizer.logger.util.ProgressMonitorInterface;
import us.ihmc.yoVariables.listener.RewoundListener;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.simulationconstructionset.PlaybackListener;

public class MultiVideoDataPlayer implements PlaybackListener, RewoundListener
{
   private final YoLong timestamp;

   
   private final ArrayList<String> videos = new ArrayList<>();

   
   private final HashMap<String, Camera> cameras = new HashMap<>();
   private final HashMap<String, VideoDataPlayer> players = new HashMap<>();

   private VideoDataPlayer activePlayer = null;

   public MultiVideoDataPlayer(File dataDirectory, LogProperties logProperties, YoLong timestamp)
   {
      this.timestamp = timestamp;

      for(int i = 0; i < logProperties.getCameras().size(); i++)
      {
         Camera camera = logProperties.getCameras().get(i);
         try
         {
            VideoDataPlayer player = new VideoDataPlayer(camera, dataDirectory, logProperties.getVideo().getHasTimebase());
            players.put(camera.getNameAsString(), player);
            cameras.put(camera.getNameAsString(), camera);
            videos.add(camera.getNameAsString());
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
   public void notifyOfIndexChange(int newIndex)
   {
      if (activePlayer != null)
      {
         activePlayer.showVideoFrame(timestamp.getLongValue());
      }
   }

   @Override
   public void notifyOfRewind()
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
         ProgressMonitorInterface monitor = new CustomProgressMonitor("Exporting " + activePlayer.getName(), selectedFile.getAbsolutePath(), 0, 100);
         monitor.setProgress(10);
         activePlayer.exportVideo(selectedFile, startTimestamp, endTimestamp, monitor);
         monitor.close();
      }
   }

   public void crop(File selectedDirectory, long startTimestamp, long endTimestamp, ProgressMonitorInterface monitor) throws IOException
   {

      for (int i = 0; i < videos.size(); i++)
      {
         String video = videos.get(i);
         Camera camera = cameras.get(video);
         if (monitor != null)
         {
            monitor.setNote("Cropping video " + video);
            monitor.setProgress(50 + ((int) (50.0 * ((double) i / (double) videos.size()))));
         }

         File timestampFile = new File(selectedDirectory, camera.getTimestampFileAsString());
         File videoFile = new File(selectedDirectory, camera.getVideoFileAsString());
         players.get(video).cropVideo(videoFile, timestampFile, startTimestamp, endTimestamp, monitor);
      }
   }

}
