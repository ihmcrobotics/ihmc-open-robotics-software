package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;

public interface VisibilityMapHolder
{
   public int getMapId();

   public VisibilityMap getVisibilityMapInLocal();

   public VisibilityMap getVisibilityMapInWorld();
   
   default double getConnectionWeight(Connection connection)
   {
      return connection.length();
   }
}
