package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

import us.ihmc.graphicsDescription.GraphicsUpdatable;

public class GraphicsUpdatablePlaybackListener implements PlaybackListener
{
   private final ArrayList<GraphicsUpdatable> graphicsUpdatableList;

   public GraphicsUpdatablePlaybackListener(ArrayList<GraphicsUpdatable> graphicsUpdatableList)
   {
      this.graphicsUpdatableList = graphicsUpdatableList;
   }

   @Override
   public void notifyOfIndexChange(int newIndex)
   {
      if (graphicsUpdatableList != null)
      {
         for (int i=0; i<graphicsUpdatableList.size(); i++)
         {
            GraphicsUpdatable graphicsUpdatable = graphicsUpdatableList.get(i);
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
