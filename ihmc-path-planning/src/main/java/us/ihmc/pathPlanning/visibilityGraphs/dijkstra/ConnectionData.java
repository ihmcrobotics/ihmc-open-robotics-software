package us.ihmc.pathPlanning.visibilityGraphs.dijkstra;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;

class ConnectionData
{
   Connection connection;
   double edgeWeight;

   @Override
   public int hashCode()
   {
      return connection.hashCode();
   }
}
