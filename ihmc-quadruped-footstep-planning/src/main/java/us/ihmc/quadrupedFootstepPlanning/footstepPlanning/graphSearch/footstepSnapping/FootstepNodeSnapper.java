package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

public abstract class FootstepNodeSnapper implements FootstepNodeSnapperReadOnly
{
   private final HashMap<SnapKey, FootstepNodeSnapData> snapDataHolder = new HashMap<>();
   protected PlanarRegionsList planarRegionsList;
   private final TIntObjectMap<List<PlanarRegion>> nearbyPlanarRegions = new TIntObjectHashMap<>();

   protected final FootstepPlannerParameters parameters;

   public FootstepNodeSnapper()
   {
      this(null);
   }

   public FootstepNodeSnapper(FootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;

      snapDataHolder.clear();
   }

   public boolean hasPlanarRegions()
   {
      return planarRegionsList != null && !planarRegionsList.isEmpty();
   }

   public FootstepNodeSnapData snapFootstepNode(FootstepNode node)
   {
      return snapFootstepNode(node.getXIndex(node.getMovingQuadrant()), node.getYIndex(node.getMovingQuadrant()));
   }

   public FootstepNodeSnapData snapFootstepNode(int xIndex, int yIndex)
   {
      SnapKey key = new SnapKey(xIndex, yIndex);
      if (snapDataHolder.containsKey(key))
      {
         return snapDataHolder.get(key);
      }
      else if (planarRegionsList == null || planarRegionsList.isEmpty())
      {
         return FootstepNodeSnapData.identityData();
      }
      else
      {
         FootstepNodeSnapData snapData = snapInternal(xIndex, yIndex);
         addSnapData(xIndex, yIndex, snapData);
         return snapData;
      }
   }


   /**
    * Can manually add snap data for a footstep node to bypass the snapper.
    */
   public void addSnapData(int xIndex, int yIndex, FootstepNodeSnapData snapData)
   {
      snapDataHolder.put(new SnapKey(xIndex, yIndex), snapData);
   }

   @Override
   public FootstepNodeSnapData getSnapData(int xIndex, int yIndex)
   {
      return snapDataHolder.get(new SnapKey(xIndex, yIndex));
   }

   protected abstract FootstepNodeSnapData snapInternal(int xIndex, int yIndex);

   private class SnapKey
   {
      private final int xIndex;
      private final int yIndex;
      private final int hashCode;

      private SnapKey(FootstepNode node)
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
