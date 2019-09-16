package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import boofcv.struct.image.Planar;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

//TODO: +++JerryPratt: Either remove this class or add more helper methods here. Right now it does not do enough worth keeping it around.
//TODO: +++JerryPratt: Clean up package structure for all of these classes.
public class NavigableRegions
{
   private List<PlanarRegion> regions;
   private List<NavigableRegion> navigableRegions;

   private final VisibilityGraphsParametersReadOnly parameters;

   public NavigableRegions(VisibilityGraphsParametersReadOnly parameters, PlanarRegionsList regions)
   {
      this(parameters, regions.getPlanarRegionsAsList());
   }

   public NavigableRegions(VisibilityGraphsParametersReadOnly parameters, List<PlanarRegion> regions)
   {
      this.parameters = parameters == null ? new DefaultVisibilityGraphParameters() : parameters;
      setPlanarRegions(regions);
   }

   public void setPlanarRegions(List<PlanarRegion> regions)
   {
      if (regions != null)
      {
         regions = PlanarRegionTools.ensureClockwiseOrder(regions);
         this.regions = regions.stream().filter(parameters.getPlanarRegionFilter()::isPlanarRegionRelevant).collect(Collectors.toList());
      }
      else
      {
         this.regions = null;
      }

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
