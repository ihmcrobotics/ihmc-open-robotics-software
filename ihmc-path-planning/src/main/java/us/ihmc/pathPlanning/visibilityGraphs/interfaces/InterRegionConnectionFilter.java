package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;

public interface InterRegionConnectionFilter
{
   default boolean isConnectionValid(Connection connection)
   {
      return isConnectionValid(connection.getSourcePoint(), connection.getTargetPoint());
   }

   /**
    *  Maximum possible 3D inter regions connection distance. Used to speed things up by using bounding boxes.
    *  If two points are further apart than this, then isConnectionValid should return false;
    */
   double getMaximumInterRegionConnetionDistance();

   boolean isConnectionValid(ConnectionPoint3D source, ConnectionPoint3D target);
}
