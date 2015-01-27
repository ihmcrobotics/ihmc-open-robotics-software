package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

import us.ihmc.utilities.GraphicsUpdatable;

public class GraphicsUpdatablePlaybackListener implements PlaybackListener
{
   private final ArrayList<GraphicsUpdatable> graphicsUpdatableList;

   public GraphicsUpdatablePlaybackListener(ArrayList<GraphicsUpdatable> graphicsUpdatableList)
   {
      this.graphicsUpdatableList = graphicsUpdatableList;
   }

   @Override
   public void indexChanged(int newIndex, double newTime)
   {
      if (graphicsUpdatableList != null)
      {
         for (GraphicsUpdatable graphicsUpdatable : graphicsUpdatableList)
         {
            graphicsUpdatable.update();
         }
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
}
