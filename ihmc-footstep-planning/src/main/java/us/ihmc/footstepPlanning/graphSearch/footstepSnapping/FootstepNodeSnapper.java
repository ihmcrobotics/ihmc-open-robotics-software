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
   private static final double proximityForPlanarRegionsNearby = 2.0;

   private final HashMap<FootstepNode, FootstepNodeSnapData> snapDataHolder = new HashMap<>();
   protected PlanarRegionsList planarRegionsList;
   private final TIntObjectMap<List<PlanarRegion>> nearbyPlanarRegions = new TIntObjectHashMap<>();
   private final TIntObjectMap<List<PlanarRegion>> bodyCollisionPlanarRegions = new TIntObjectHashMap<>();
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
      bodyCollisionPlanarRegions.clear();
      nearbyNavigablePlanarRegions.clear();
   }

   public boolean hasPlanarRegions()
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

   public List<PlanarRegion> getOrCreateBodyCollisionRegions(double unroundedX, double unroundedY, double groundHeight)
   {
      double roundedX = FootstepNode.round(unroundedX);
      double roundedY = FootstepNode.round(unroundedY);
      int hashcode = FootstepNode.computePlanarRegionsHashCode(roundedX, roundedY);

      if (bodyCollisionPlanarRegions.containsKey(hashcode))
         return bodyCollisionPlanarRegions.get(hashcode);

      List<PlanarRegion> nearbyRegions = getOrCreateNearbyRegions(roundedX, roundedY);

      Point2DReadOnly centerPoint = new Point2D(roundedX, roundedY);

      double maxDistanceForCollision = 1.25 * Math
            .sqrt(parameters.getBodyBoxDepth() * parameters.getBodyBoxDepth() + parameters.getBodyBoxWidth() * parameters.getBodyBoxWidth());

      List<PlanarRegion> bodyProximityRegions;
      if (maxDistanceForCollision < proximityForPlanarRegionsNearby)
         bodyProximityRegions = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(centerPoint, maxDistanceForCollision, nearbyRegions);
      else
         bodyProximityRegions = nearbyRegions;

      double minHeight = parameters.getBodyBoxBaseZ();
      double maxHeight = parameters.getBodyBoxBaseZ() + parameters.getBodyBoxHeight();

      List<PlanarRegion> bodyCollisionRegions = bodyProximityRegions.stream().filter(
            region -> parameters.getBodyCollisionRegionFilter().isPlanarRegionCollidable(region, groundHeight, minHeight, maxHeight))
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
