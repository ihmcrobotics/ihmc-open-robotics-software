package us.ihmc.pathPlanning.visibilityGraphs.dijkstra;

import java.util.Comparator;
import java.util.HashMap;

import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;

class ConnectionPointComparator implements Comparator<ConnectionPoint3D>
{
   private final HashMap<ConnectionPoint3D, Double> nodeCosts;

   ConnectionPointComparator(HashMap<ConnectionPoint3D, Double> nodeCosts)
   {
      this.nodeCosts = nodeCosts;
   }

   @Override
   public int compare(ConnectionPoint3D point1, ConnectionPoint3D point2)
   {
      double cost1 = nodeCosts.get(point1);
      double cost2 = nodeCosts.get(point2);
      if (cost1 == cost2)
         return 0;
      return cost1 < cost2 ? -1 : 1;
   }
}
