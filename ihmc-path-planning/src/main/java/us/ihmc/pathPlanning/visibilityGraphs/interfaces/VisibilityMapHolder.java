package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.pathPlanning.visibilityGraphs.VisibilityMap;

public interface VisibilityMapHolder
{
   public int getMapId();

   public VisibilityMap getVisibilityMapInLocal();

   public VisibilityMap getVisibilityMapInWorld();
}
