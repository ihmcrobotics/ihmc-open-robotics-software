package us.ihmc.simulationconstructionsettools1.bambooTools.videoWall;

import java.awt.GridLayout;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JPanel;

class VideoPlayerGrid extends JPanel
{
   private static final long serialVersionUID = 8540205804417525709L;
   
   private final List<List<VideoPlayer>> videoPlayerGrid = new ArrayList<>();
   private final List<VideoPlayer> listOfVideoPlayers = new ArrayList<>();
   private final int numberOfColumnsOfVideo;

   public VideoPlayerGrid(int numberOfRowsOfVideo, int numberOfColumnsOfVideo)
   {
      this.numberOfColumnsOfVideo = numberOfColumnsOfVideo;
      this.setLayout(new GridLayout(numberOfRowsOfVideo, numberOfColumnsOfVideo));
      
      for (int i = 0; i < numberOfRowsOfVideo; i++)
      {
         videoPlayerGrid.add(new ArrayList<VideoPlayer>());
         
         for (int j = 0; j < numberOfColumnsOfVideo; j++)
         {
            VideoPlayer videoPlayer = new VideoPlayer();
            videoPlayerGrid.get(i).add(videoPlayer);
            add(videoPlayer.getjPanel());
            listOfVideoPlayers.add(videoPlayer);
         }
      }
   }

   public void updateVideoPath(int index, Path pathToVideo)
   {
      videoPlayerGrid.get(index / numberOfColumnsOfVideo).get(index % numberOfColumnsOfVideo).updateVideoPath(pathToVideo);
   }

   public List<VideoPlayer> getVideoPlayers()
   {      
      return listOfVideoPlayers;
   }
}