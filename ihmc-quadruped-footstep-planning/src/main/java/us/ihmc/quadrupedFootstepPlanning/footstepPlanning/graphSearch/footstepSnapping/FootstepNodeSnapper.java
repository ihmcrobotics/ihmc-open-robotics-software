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

import java.util.List;
import java.util.stream.Collectors;

public abstract class FootstepNodeSnapper implements FootstepNodeSnapperReadOnly
{
   private static final double proximityForPlanarRegionsNearby = 2.0;

   private final TIntObjectHashMap<FootstepNodeSnapData> snapDataHolder = new TIntObjectHashMap<>();
   protected PlanarRegionsList planarRegionsList;
   private final TIntObjectMap<List<PlanarRegion>> nearbyPlanarRegions = new TIntObjectHashMap<>();
   private final TIntObjectMap<List<PlanarRegion>> nearbyNavigablePlanarRegions = new TIntObjectHashMap<>();

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
      nearbyPlanarRegions.clear();
      nearbyNavigablePlanarRegions.clear();
   }

   public boolean hasPlanarRegions()
   {
      return planarRegionsList != null && !planarRegionsList.isEmpty();
   }

   public FootstepNodeSnapData snapFootstepNode(FootstepNode footstepNode)
   {
      if (snapDataHolder.containsKey(footstepNode.getSnapDataHashCode()))
      {
         return snapDataHolder.get(footstepNode.getSnapDataHashCode());
      }
      else if (planarRegionsList == null || planarRegionsList.isEmpty())
//      if (planarRegionsList == null || planarRegionsList.isEmpty())
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

   public List<PlanarRegion> getOrCreateNearbyRegions(double roundedX, double roundedY)
   {
      int hashCode = FootstepNode.computePlanarRegionsHashCode(roundedX, roundedY);
      if (nearbyPlanarRegions.containsKey(hashCode))
         return nearbyPlanarRegions.get(hashCode);

      Point2DReadOnly centerPoint = new Point2D(roundedX, roundedY);
      List<PlanarRegion> nearbyRegions = PlanarRegionTools
            .filterPlanarRegionsWithBoundingCircle(centerPoint, proximityForPlanarRegionsNearby, planarRegionsList.getPlanarRegionsAsList());
      nearbyPlanarRegions.put(hashCode, nearbyRegions);

      return nearbyRegions;
   }

   public List<PlanarRegion> getOrCreateSteppableRegions(double roundedX, double roundedY)
   {
      int hashcode = FootstepNode.computePlanarRegionsHashCode(roundedX, roundedY);

      if (nearbyNavigablePlanarRegions.containsKey(hashcode))
         return nearbyNavigablePlanarRegions.get(hashcode);

      List<PlanarRegion> nearbyRegions = getOrCreateNearbyRegions(roundedX, roundedY);

      if (parameters == null)
         return nearbyRegions;

      List<PlanarRegion> navigableRegions = nearbyRegions.stream().filter(region -> parameters.getSteppableRegionFilter().isPlanarRegionSteppable(region))
                                                         .collect(Collectors.toList());
      nearbyNavigablePlanarRegions.put(hashcode, navigableRegions);

      return navigableRegions;
   }


   /**
    * Can manually add snap data for a footstep node to bypass the snapper.
    */
   public void addSnapData(FootstepNode footstepNode, FootstepNodeSnapData snapData)
   {
      snapDataHolder.put(footstepNode.getSnapDataHashCode(), snapData);
   }

   @Override
   public FootstepNodeSnapData getSnapData(FootstepNode node)
   {
      return snapDataHolder.get(node.hashCode());
   }

   protected abstract FootstepNodeSnapData snapInternal(FootstepNode footstepNode);
}
