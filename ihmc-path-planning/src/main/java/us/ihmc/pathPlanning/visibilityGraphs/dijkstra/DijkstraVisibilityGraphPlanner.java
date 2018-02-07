package us.ihmc.pathPlanning.visibilityGraphs.dijkstra;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

import java.util.*;

public class DijkstraVisibilityGraphPlanner
{
   private final HashMap<ConnectionPoint3D, HashSet<ConnectionData>> visibilityMap = new HashMap<>();
   private final HashMap<ConnectionPoint3D, Double> nodeCosts = new HashMap<>();
   private final HashMap<ConnectionPoint3D, ConnectionData> incomingBestEdge = new HashMap<>();

   private PriorityQueue<ConnectionPoint3D> stack;
   private ConnectionPoint3D closestPointToGoal;
   private ConnectionPoint3D startPoint, goalPoint;

   public void setVisibilityMap(Collection<VisibilityMapHolder> visibilityMapHolders)
   {
      visibilityMap.clear();
      nodeCosts.clear();

      for (VisibilityMapHolder visibilityMapHolder : visibilityMapHolders)
      {
         VisibilityMap visibilityMap = visibilityMapHolder.getVisibilityMapInWorld();
         for (Connection connection : visibilityMap)
         {
            ConnectionData connectionData = new ConnectionData();
            connectionData.connection = connection;
            connectionData.edgeWeight = visibilityMapHolder.getConnectionWeight(connection);

            this.visibilityMap.computeIfAbsent(connection.getSourcePoint(), (p) -> new HashSet<>()).add(connectionData);
            this.visibilityMap.computeIfAbsent(connection.getTargetPoint(), (p) -> new HashSet<>()).add(connectionData);
         }
      }
   }

   public void initialize(ConnectionPoint3D startPoint, ConnectionPoint3D goalPoint)
   {
      this.startPoint = startPoint;
      this.goalPoint = goalPoint;

      closestPointToGoal = startPoint;
      nodeCosts.put(startPoint, 0.0);
      stack = new PriorityQueue<>(new ConnectionPointComparator(nodeCosts));
      stack.add(startPoint);
   }

   /**
    * Plans a path over the visibility maps given by {@link #setVisibilityMap}
    * @return shortest path to the goal if one exists. Otherwise it will return a path to the
    * closest possible node to the goal
    */
   public List<Point3DReadOnly> plan()
   {
      while(!stack.isEmpty())
      {
         ConnectionPoint3D sourcePoint = stack.poll();
         HashSet<ConnectionData> connections = visibilityMap.getOrDefault(sourcePoint, new HashSet<>());
         if(sourcePoint.distance(goalPoint) < closestPointToGoal.distance(goalPoint))
            closestPointToGoal = sourcePoint;

         for (ConnectionData connectionData : connections)
         {
            Connection connection = connectionData.connection;
            if(connection.getTargetPoint().equals(sourcePoint))
               connection.flip();
            ConnectionPoint3D targetPoint = connection.getTargetPoint();

            double nodeCost = nodeCosts.get(sourcePoint) + connectionData.edgeWeight;

            if(!nodeCosts.containsKey(targetPoint) || nodeCosts.get(targetPoint) > nodeCost)
            {
               nodeCosts.put(targetPoint, nodeCost);
               incomingBestEdge.put(targetPoint, connectionData);
            }

            stack.add(targetPoint);
         }
      }

      if(nodeCosts.containsKey(goalPoint))
      {
         return getPathToPoint(goalPoint);
      }
      else
      {
         return getPathToPoint(closestPointToGoal);
      }
   }

   public List<Point3DReadOnly> getPathToPoint(ConnectionPoint3D point)
   {
      List<Point3DReadOnly> path = new ArrayList<>();
      path.add(point);

      Connection incomingEdge = incomingBestEdge.get(point).connection;
      while(incomingEdge != null)
      {
         ConnectionPoint3D sourcePoint = incomingEdge.getSourcePoint();
         path.add(sourcePoint);
         incomingEdge = incomingBestEdge.get(sourcePoint).connection;
      }

      Collections.reverse(path);
      return path;
   }
}
