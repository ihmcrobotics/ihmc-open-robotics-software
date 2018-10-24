package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public abstract class FootstepNodeSnapper implements FootstepNodeSnapperReadOnly
{
   private static final double proximityForPlanarRegions = 1.25;

   private final HashMap<FootstepNode, FootstepNodeSnapData> snapDataHolder = new HashMap<>();
   protected PlanarRegionsList planarRegionsList;
   protected final TIntObjectMap<List<PlanarRegion>> nearbyPlanarRegions = new TIntObjectHashMap<>();
   protected final TIntObjectMap<List<PlanarRegion>> bodyCollisionPlanarRegions = new TIntObjectHashMap<>();
   protected final TIntObjectMap<List<PlanarRegion>> nearbyNavigablePlanarRegions = new TIntObjectHashMap<>();

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
      nearbyPlanarRegions.clear();
      snapDataHolder.clear();
   }

   public TIntObjectMap<List<PlanarRegion>> getNearbyPlanarRegions()
   {
      return nearbyPlanarRegions;
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

         getOrCreateNearbyRegions(footstepNode.getRoundedX(), footstepNode.getRoundedY());

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
            .filterPlanarRegionsWithBoundingCircle(centerPoint, proximityForPlanarRegions, planarRegionsList.getPlanarRegionsAsList());
      nearbyPlanarRegions.put(hashCode, nearbyRegions);

      if (parameters != null)
      {
         List<PlanarRegion> navigableRegions = nearbyRegions.stream().filter(
               region -> parameters.getNavigableRegionFilter().isPlanarRegionNavigable(region, nearbyRegions)).collect(Collectors.toList());
         nearbyNavigablePlanarRegions.put(hashCode, navigableRegions);
      }

      return nearbyRegions;
   }

   public List<PlanarRegion> getOrCreateBodyCollisionRegions(double unroundedX, double unroundedY, double groundHeight)
   {
      double roundedX = FootstepNode.round(unroundedX);
      double roundedY = FootstepNode.round(unroundedY);
      int hashcode = FootstepNode.computePlanarRegionsHashCode(roundedX, roundedY);

      if (bodyCollisionPlanarRegions.containsKey(hashcode))
         return bodyCollisionPlanarRegions.get(hashcode);

      List<PlanarRegion> nearbyRegions = getOrCreateNearbyRegions(roundedX, roundedY);

      double minHeight = parameters.getBodyBoxCenterHeight() - 0.5 * parameters.getBodyBoxCenterHeight();
      List<PlanarRegion> bodyCollisionRegions = nearbyRegions.stream().filter(region -> region.getBoundingBox3dInWorld().getMinZ() - groundHeight >= minHeight)
                                                             .collect(Collectors.toList());

      bodyCollisionPlanarRegions.put(FootstepNode.computePlanarRegionsHashCode(roundedX, roundedY), bodyCollisionRegions);

      return bodyCollisionRegions;
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
