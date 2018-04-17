package us.ihmc.robotEnvironmentAwareness.ui;

import boofcv.misc.UnsupportedException;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOccupancyOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeNodeMessage;
import us.ihmc.robotics.geometry.PlanarRegion;

public class UIOcTreeNode extends AbstractOccupancyOcTreeNode<UIOcTreeNode>
{
   private int regionId = PlanarRegion.NO_REGION_ID;
   private final float normalX;
   private final float normalY;
   private final float normalZ;
   private final float normalAverageDeviation;
   private final int normalConsensusSize;
   private final float hitLocationX;
   private final float hitLocationY;
   private final float hitLocationZ;

   private final long numberOfHits;

   // Empty constructor is needed for the NodeBuilder.
   public UIOcTreeNode()
   {
      normalX = Float.NaN;
      normalY = Float.NaN;
      normalZ = Float.NaN;
      normalAverageDeviation = Float.NaN;
      normalConsensusSize = 0;
      hitLocationX = Float.NaN;
      hitLocationY = Float.NaN;
      hitLocationZ = Float.NaN;
      numberOfHits = 0;
   }

   public UIOcTreeNode(NormalOcTreeNodeMessage normalOcTreeNodeMessage, double resolution, int treeDepth)
   {
      int k0 = normalOcTreeNodeMessage.key0;
      int k1 = normalOcTreeNodeMessage.key1;
      int k2 = normalOcTreeNodeMessage.key2;
      int depth = normalOcTreeNodeMessage.depth;
      setProperties(k0, k1, k2, depth, resolution, treeDepth);

      normalX = normalOcTreeNodeMessage.normalX;
      normalY = normalOcTreeNodeMessage.normalY;
      normalZ = normalOcTreeNodeMessage.normalZ;
      normalAverageDeviation = normalOcTreeNodeMessage.normalAverageDeviation;
      normalConsensusSize = normalOcTreeNodeMessage.normalConsensusSize;
      hitLocationX = normalOcTreeNodeMessage.hitLocationX;
      hitLocationY = normalOcTreeNodeMessage.hitLocationY;
      hitLocationZ = normalOcTreeNodeMessage.hitLocationZ;
      numberOfHits = normalOcTreeNodeMessage.numberOfHits;

      if (normalOcTreeNodeMessage.getNumberOfChildren() > 0)
      {
         super.allocateChildren();

         for (int childIndex = 0; childIndex < 8; childIndex++)
         {
            NormalOcTreeNodeMessage otherChild = normalOcTreeNodeMessage.children[childIndex];
            if (otherChild != null)
            {
               children[childIndex] = new UIOcTreeNode(otherChild, resolution, treeDepth);
            }
         }
      }
   }

   public boolean isNormalSet()
   {
      return !Float.isNaN(normalX) && !Float.isNaN(normalY) && !Float.isNaN(normalZ);
   }

   public boolean isHitLocationSet()
   {
      return !Float.isNaN(hitLocationX) && !Float.isNaN(hitLocationY) && !Float.isNaN(hitLocationZ);
   }

   public void setRegionId(int regionId)
   {
      this.regionId = regionId;
   }

   public void setRegionIdFromChildren()
   {
      regionId = computeRegionIdFromChildren();
   }

   public boolean isPartOfRegion()
   {
      return regionId != PlanarRegion.NO_REGION_ID;
   }

   public int computeRegionIdFromChildren()
   {
      if (!hasAtLeastOneChild())
      {
         return PlanarRegion.NO_REGION_ID;
      }

      int indexRegionWithHighestCount = -1;
      int highestCount = -1;

      for (int i = 0; i < 8; i++)
      {
         UIOcTreeNode currentChild = children[i];
         if (currentChild != null && currentChild.isPartOfRegion())
         {
            int currentCount = 1;
            
            for (int j = 0; j < i; j++)
            {
               UIOcTreeNode other = children[j];
               if (other != null && currentChild.getRegionId() == other.getRegionId())
                  currentCount++;
            }

            if (indexRegionWithHighestCount < 0 || currentCount > highestCount)
            {
               indexRegionWithHighestCount = i;
               highestCount = currentCount;
            }
         }
      }

      if (indexRegionWithHighestCount < 0)
         return PlanarRegion.NO_REGION_ID;
      else
         return children[indexRegionWithHighestCount].regionId;
   }

   public void getNormal(Vector3D normalToPack)
   {
      normalToPack.set(normalX, normalY, normalZ);
   }

   public float getNormalAverageDeviation()
   {
      return normalAverageDeviation;
   }

   public int getNormalConsensusSize()
   {
      return normalConsensusSize;
   }

   public void getHitLocation(Point3D hitLocationToPack)
   {
      hitLocationToPack.set(hitLocationX, hitLocationY, hitLocationZ);
   }

   public long getNumberOfHits()
   {
      return numberOfHits;
   }

   public int getRegionId()
   {
      return regionId;
   }

   @Override
   protected void clear()
   {
      throw new UnsupportedException();
   }

   @Override
   public void addValue(float logOdds)
   {
      throw new UnsupportedException();
   }

   @Override
   public void allocateChildren()
   {
      throw new UnsupportedException();
   }
}
