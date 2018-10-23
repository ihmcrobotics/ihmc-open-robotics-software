package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public abstract class FootstepNodeSnapper implements FootstepNodeSnapperReadOnly
{
   private static final double proximityForPlanarRegions = 2.5;

   private final HashMap<FootstepNode, FootstepNodeSnapData> snapDataHolder = new HashMap<>();
   protected PlanarRegionsList planarRegionsList;
   protected final TIntObjectMap<List<PlanarRegion>> nearbyPlanarRegions = new TIntObjectHashMap<>();
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
         if (!nearbyPlanarRegions.containsKey(footstepNode.getPlanarRegionsHashCode()))
         {
            Point2DReadOnly roundedPoint = new Point2D(footstepNode.getRoundedX(), footstepNode.getRoundedY());
            List<PlanarRegion> nearbyRegions = PlanarRegionTools
                  .filterPlanarRegionsWithBoundingCircle(roundedPoint, proximityForPlanarRegions, planarRegionsList.getPlanarRegionsAsList());
            nearbyPlanarRegions.put(footstepNode.getPlanarRegionsHashCode(), nearbyRegions);

            if (parameters != null)
            {
               List<PlanarRegion> navigableRegions = nearbyRegions.stream().filter(
                     region -> parameters.getNavigableRegionFilter().isPlanarRegionNavigable(region, nearbyRegions)).collect(Collectors.toList());
               nearbyNavigablePlanarRegions.put(footstepNode.getPlanarRegionsHashCode(), navigableRegions);
            }
         }

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
