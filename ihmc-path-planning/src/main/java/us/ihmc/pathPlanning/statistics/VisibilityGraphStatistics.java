package us.ihmc.pathPlanning.statistics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;

public class VisibilityGraphStatistics implements PlannerStatistics<VisibilityGraphStatistics>
{
   private int startMapId = -1;
   private int goalMapId = -1;
   private int interRegionsMapId = -1;

   private VisibilityMap startMap = new VisibilityMap();
   private VisibilityMap goalMap = new VisibilityMap();
   private VisibilityMap interRegionsMap = new VisibilityMap();

   private final List<VisibilityMapWithNavigableRegion> visibilityMapsWithNavigableRegions = new ArrayList<>();

   @Override
   public StatisticsType getStatisticsType()
   {
      return StatisticsType.VISIBILITY_GRAPH;
   }

   @Override
   public void set(VisibilityGraphStatistics other)
   {
      setStartVisibilityMapInWorld(other.startMapId, other.startMap);
      setGoalVisibilityMapInWorld(other.goalMapId, other.goalMap);
      setInterRegionsVisibilityMapInWorld(other.interRegionsMapId, other.interRegionsMap);
      setVisibilityMapsWithNavigableRegions(other.visibilityMapsWithNavigableRegions);
   }

   public void setStartMapId(int mapId)
   {
      startMapId = mapId;
   }

   public void setGoalMapId(int mapId)
   {
      goalMapId = mapId;
   }

   public void setStartVisibilityMapInWorld(int mapId, VisibilityMap startMap)
   {
      startMapId = mapId;
      this.startMap = startMap;
   }

   public void setGoalVisibilityMapInWorld(int mapId, VisibilityMap goalMap)
   {
      this.goalMapId = mapId;
      this.goalMap = goalMap;
   }

   public void setInterRegionsVisibilityMapInWorld(int mapId, VisibilityMap interRegionsMap)
   {
      this.interRegionsMapId = mapId;
      this.interRegionsMap = interRegionsMap;
   }

   public void setVisibilityMapsWithNavigableRegions(List<VisibilityMapWithNavigableRegion> navigableRegions)
   {
      this.visibilityMapsWithNavigableRegions.clear();
      addNavigableRegions(navigableRegions);
   }

   public void addNavigableRegions(List<VisibilityMapWithNavigableRegion> navigableRegions)
   {
      for (int i = 0; i < navigableRegions.size(); i++)
         addNavigableRegion(navigableRegions.get(i));
   }

   public void addNavigableRegion(VisibilityMapWithNavigableRegion navigableRegion)
   {
      visibilityMapsWithNavigableRegions.add(navigableRegion);
   }

   public VisibilityMap getGoalVisibilityMap()
   {
      return goalMap;
   }

   public VisibilityMap getStartVisibilityMap()
   {
      return startMap;
   }

   public VisibilityMap getInterRegionsVisibilityMap()
   {
      return interRegionsMap;
   }

   public int getGoalMapId()
   {
      return goalMapId;
   }

   public int getStartMapId()
   {
      return startMapId;
   }

   public int getInterRegionsMapId()
   {
      return interRegionsMapId;
   }

   public int getNumberOfNavigableRegions()
   {
      return visibilityMapsWithNavigableRegions.size();
   }

   public VisibilityMapWithNavigableRegion getNavigableRegion(int regionNumber)
   {
      return visibilityMapsWithNavigableRegions.get(regionNumber);
   }
}
