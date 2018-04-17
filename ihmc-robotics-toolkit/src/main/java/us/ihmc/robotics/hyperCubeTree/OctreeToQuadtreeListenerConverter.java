package us.ihmc.robotics.hyperCubeTree;

import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundListener;

public class OctreeToQuadtreeListenerConverter implements HyperCubeTreeListener<Boolean, Void>
{
   private final QuadTreeForGroundListener quadListener;

   public OctreeToQuadtreeListenerConverter(QuadTreeForGroundListener quadListener)
   {
      this.quadListener = quadListener;
   }

   public void nodeAdded(String id, OneDimensionalBounds[] bounds, HyperCubeLeaf<Boolean> leaf)
   {
      Box boundryBox = new Box(bounds[0].min(), bounds[1].min(), bounds[0].max(), bounds[1].max());
      if (leaf != null)
         if (leaf.getValue())
            quadListener.nodeAdded(id, boundryBox, (float) leaf.getLocation()[0], (float) leaf.getLocation()[1], (float) leaf.getLocation()[2]);
   }

   public void nodeRemoved(String id)
   {
      quadListener.nodeRemoved(id);
   }

   public void leafAdded(HyperCubeLeaf<Boolean> leaf)
   {
      if (leaf != null)
         if (leaf.getValue())
         {
            double[] location = leaf.getLocation();
            quadListener.RawPointAdded((float) location[0], (float) location[1], (float) location[2]);
         }
   }

   public void treeCleared()
   {

   }

   public void metaDataUpdated(String id, OneDimensionalBounds[] bounds, Void data)
   {
   }

}
