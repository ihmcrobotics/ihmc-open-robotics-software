package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
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

   public List<NavigableRegion> getNavigableRegions()
   {
      return navigableRegions;
   }

   public Point3DReadOnly[][] getNavigableExtrusions()
   {
      Point3DReadOnly[][] allNavigableExtrusions = new Point3D[navigableRegions.size()][];

      for (int i = 0; i < navigableRegions.size(); i++)
      {
         NavigableRegion localPlanner = navigableRegions.get(i);
         Point3DReadOnly[] navigableExtrusions = new Point3D[localPlanner.getAllClusters().size()];

         for (Cluster cluster : localPlanner.getAllClusters())
         {
            for (int j = 0; j < cluster.getNumberOfNavigableExtrusions(); j++)
            {
               navigableExtrusions[j] = cluster.getNavigableExtrusionInWorld(j);
            }
         }

         allNavigableExtrusions[i] = navigableExtrusions;
      }

      return allNavigableExtrusions;
   }

   public void filterPlanarRegionsWithBoundingCapsule(Point3DReadOnly start, Point3DReadOnly goal, double explorationDistanceFromStartGoal)
   {
      regions = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(start, goal, explorationDistanceFromStartGoal, regions);
   }

   public void createNavigableRegions()
   {
      navigableRegions = VisibilityGraphsFactory.createNavigableRegionButNotVisibilityMaps(regions, parameters);
   }

   public List<NavigableRegion> getNaviableRegionsList()
   {
      return navigableRegions;
   }

}
