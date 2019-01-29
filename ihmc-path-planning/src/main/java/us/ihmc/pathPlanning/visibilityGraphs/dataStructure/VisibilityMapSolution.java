package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.ArrayList;

import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegions;

public class VisibilityMapSolution
{
   private NavigableRegions navigableRegions;
   private final ArrayList<VisibilityMapWithNavigableRegion> visibilityMapsWithNavigableRegions = new ArrayList<>();
   private InterRegionVisibilityMap interRegionVisibilityMap;
   private SingleSourceVisibilityMap startMap, goalMap;

   public VisibilityMapSolution()
   {
   }

   public void setNavigableRegions(NavigableRegions navigableRegions)
   {
      this.navigableRegions = navigableRegions;
   }

   public void setVisibilityMapsWithNavigableRegions(ArrayList<VisibilityMapWithNavigableRegion> visibilityMapsWithNavigableRegions)
   {
      this.visibilityMapsWithNavigableRegions.clear();
      this.visibilityMapsWithNavigableRegions.addAll(visibilityMapsWithNavigableRegions);
   }

   public void setInterRegionVisibilityMap(InterRegionVisibilityMap interRegionVisibilityMap)
   {
      this.interRegionVisibilityMap = interRegionVisibilityMap;
   }

   public void setStartMap(SingleSourceVisibilityMap startMap)
   {
      this.startMap = startMap;
   }

   public void setGoalMap(SingleSourceVisibilityMap goalMap)
   {
      this.goalMap = goalMap;
   }

   public NavigableRegions getNavigableRegions()
   {
      return navigableRegions;
   }

   public ArrayList<VisibilityMapWithNavigableRegion> getVisibilityMapsWithNavigableRegions()
   {
      return visibilityMapsWithNavigableRegions;
   }

   public InterRegionVisibilityMap getInterRegionVisibilityMap()
   {
      return interRegionVisibilityMap;
   }

   public SingleSourceVisibilityMap getStartMap()
   {
      return startMap;
   }

   public SingleSourceVisibilityMap getGoalMap()
   {
      return goalMap;
   }

}
