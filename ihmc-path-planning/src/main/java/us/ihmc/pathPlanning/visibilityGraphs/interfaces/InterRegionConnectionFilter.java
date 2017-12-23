package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;

public interface InterRegionConnectionFilter
{
   default boolean isConnectionValid(Connection connection)
   {
      return isConnectionValid(connection.getSourcePoint(), connection.getTargetPoint());
   }

   boolean isConnectionValid(ConnectionPoint3D source, ConnectionPoint3D target);
}
