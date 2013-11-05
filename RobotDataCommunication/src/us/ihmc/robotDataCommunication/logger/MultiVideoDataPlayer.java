package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.util.HashMap;

import com.yobotics.simulationconstructionset.LongYoVariable;
import com.yobotics.simulationconstructionset.PlaybackListener;
import com.yobotics.simulationconstructionset.SimulationRewoundListener;

public class MultiVideoDataPlayer implements PlaybackListener, SimulationRewoundListener
{
   private final LongYoVariable timestamp;
   
   private final String[] videos; 
   
   private final HashMap<String, VideoDataPlayer> players = new HashMap<>();
   
   private VideoDataPlayer activePlayer = null;
   
   public MultiVideoDataPlayer(File dataDirectory, LogProperties logProperties, LongYoVariable timestamp)
   {
      this.timestamp = timestamp;
      this.videos = logProperties.getVideoFiles();
      
      for(String video : videos)
      {
         VideoDataPlayer player = new VideoDataPlayer(video, dataDirectory, logProperties);
         players.put(video, player);
      }
      
      if(videos.length > 0)
      {
         setActivePlayer(videos[0]);
      }
      
   }

   public String[] getVideos()
   {
      return videos;
   }
   
   @Override
   public void indexChanged(int newIndex, double newTime)
   {
      if(activePlayer != null)
      {
         activePlayer.showVideoFrame(timestamp.getLongValue());
      }
   }

   @Override
   public void simulationWasRewound()
   {
      if(activePlayer != null)
      {
         activePlayer.showVideoFrame(timestamp.getLongValue());
      }
   }

   public void setActivePlayer(String name)
   {
      if(activePlayer != null)
      {
         activePlayer.setVisible(false);
      }
      
      if(name != null)
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
      if(activePlayer != null)
      {
         activePlayer.exportVideo(selectedFile, startTimestamp, endTimestamp);
      }
   }

}
