package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.filters.BodyCollisionRegionFilter;
import us.ihmc.footstepPlanning.filters.SteppableRegionFilter;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public abstract class FootstepNodeSnapper implements FootstepNodeSnapperReadOnly
{
   private final HashMap<FootstepNode, FootstepNodeSnapData> snapDataHolder = new HashMap<>();
   protected PlanarRegionsList planarRegionsList;

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      snapDataHolder.clear();
   }

   boolean hasPlanarRegions()
   {
      return planarRegionsList != null && !planarRegionsList.isEmpty();
   }

   public FootstepNodeSnapData snapFootstepNode(FootstepNode footstepNode)
   {
      if (snapDataHolder.containsKey(footstepNode))
      {
         return snapDataHolder.get(footstepNode);
      }
      else if (planarRegionsList == null || planarRegionsList.isEmpty())
      {
         return FootstepNodeSnapData.identityData();
      }
      else
      {
         FootstepNodeSnapData snapData = snapInternal(footstepNode);
         addSnapData(footstepNode, snapData);
         return snapData;
      }
   }

   /**
    * Can manually add snap data for a footstep node to bypass the snapper.
    */
   public void addSnapData(FootstepNode footstepNode, FootstepNodeSnapData snapData)
   {
      snapDataHolder.put(footstepNode, snapData);
   }

   @Override
   public FootstepNodeSnapData getSnapData(FootstepNode node)
   {
      return snapDataHolder.get(node);
   }

   protected abstract FootstepNodeSnapData snapInternal(FootstepNode footstepNode);
}
