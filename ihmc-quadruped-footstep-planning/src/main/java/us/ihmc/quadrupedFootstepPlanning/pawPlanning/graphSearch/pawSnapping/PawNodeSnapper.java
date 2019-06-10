package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.HashMap;

public abstract class PawNodeSnapper implements PawNodeSnapperReadOnly
{
   private final HashMap<SnapKey, PawNodeSnapData> snapDataHolder = new HashMap<>();
   protected PlanarRegionsList planarRegionsList;

   protected final PawStepPlannerParametersReadOnly parameters;

   public PawNodeSnapper()
   {
      this(null);
   }

   public PawNodeSnapper(PawStepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;

      snapDataHolder.clear();
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public boolean hasPlanarRegions()
   {
      return planarRegionsList != null && !planarRegionsList.isEmpty();
   }

   public PawNodeSnapData snapPawNode(PawNode node)
   {
      return snapPawNode(node.getMovingQuadrant(), node.getXIndex(node.getMovingQuadrant()), node.getYIndex(node.getMovingQuadrant()));
   }

   public PawNodeSnapData snapPawNode(RobotQuadrant movingQuadrant, int xIndex, int yIndex)
   {
      SnapKey key = new SnapKey(xIndex, yIndex);
      if (snapDataHolder.containsKey(key))
      {
         return snapDataHolder.get(key);
      }
      else if (planarRegionsList == null || planarRegionsList.isEmpty())
      {
         return PawNodeSnapData.identityData();
      }
      else
      {
         PawNodeSnapData snapData = snapInternal(movingQuadrant, xIndex, yIndex);
         addSnapData(xIndex, yIndex, snapData);
         return snapData;
      }
   }


   /**
    * Can manually add snap data for a paw node to bypass the snapper.
    */
   public void addSnapData(int xIndex, int yIndex, PawNodeSnapData snapData)
   {
      snapDataHolder.put(new SnapKey(xIndex, yIndex), snapData);
   }

   @Override
   public PawNodeSnapData getSnapData(int xIndex, int yIndex)
   {
      return snapDataHolder.get(new SnapKey(xIndex, yIndex));
   }

   protected abstract PawNodeSnapData snapInternal(RobotQuadrant movingQuadrant, int xIndex, int yIndex);

   private class SnapKey
   {
      private final int xIndex;
      private final int yIndex;
      private final int hashCode;

      private SnapKey(PawNode node)
      {
         this(node.getXIndex(node.getMovingQuadrant()), node.getYIndex(node.getMovingQuadrant()));
      }

      private SnapKey(int xIndex, int yIndex)
      {
         this.xIndex = xIndex;
         this.yIndex = yIndex;
         hashCode = computeHashCode(xIndex, yIndex);
      }

      @Override
      public int hashCode()
      {
         return hashCode;
      }

      private int computeHashCode(int xIndex, int yIndex)
      {
         final int prime = 31;
         int result = 1;
         result = prime * result + xIndex;
         result = prime * result + yIndex;
         return result;
      }

      @Override
      public boolean equals(Object obj)
      {
         if (this == obj)
            return true;
         if (obj == null)
            return false;
         if (getClass() != obj.getClass())
            return false;
         SnapKey other = (SnapKey) obj;

         if (xIndex != other.xIndex)
            return false;

         return yIndex == other.yIndex;
      }
   }
}
