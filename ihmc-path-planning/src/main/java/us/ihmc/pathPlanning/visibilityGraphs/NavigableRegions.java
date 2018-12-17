package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class NavigableRegions
{
   private List<PlanarRegion> regions;
   private List<NavigableRegion> navigableRegions;

   private final VisibilityGraphsParameters parameters;

   public NavigableRegions(VisibilityGraphsParameters parameters, List<PlanarRegion> regions)
   {
      this.parameters = parameters == null ? new DefaultVisibilityGraphParameters() : parameters;
      setPlanarRegions(regions);
   }

   public void setPlanarRegions(List<PlanarRegion> regions)
   {
      if (regions != null)
      {
         regions = PlanarRegionTools.ensureClockwiseOrder(regions);
         regions = regions.stream().filter(parameters.getPlanarRegionFilter()::isPlanarRegionRelevant).collect(Collectors.toList());
      }

      this.regions = regions;
   }

   public void filterPlanarRegionsWithBoundingCapsule(Point3DReadOnly start, Point3DReadOnly goal, double explorationDistanceFromStartGoal)
   {
      regions = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(start, goal, explorationDistanceFromStartGoal, regions);
   }

   public void createNavigableRegions()
   {
      navigableRegions = NavigableRegionsFactory.createNavigableRegions(regions, parameters);
   }

   public List<NavigableRegion> getNaviableRegionsList()
   {
      return navigableRegions;
   }

}
