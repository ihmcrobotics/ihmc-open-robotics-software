package us.ihmc.pathPlanning.visibilityGraphs.dijkstra;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

import java.util.*;

public class DijkstraVisibilityGraphPlanner implements VisibilityGraphPathPlanner
{
   private final HashMap<ConnectionPoint3D, HashSet<ConnectionData>> visibilityMap = new HashMap<>();
   private final HashMap<ConnectionPoint3D, Double> nodeCosts = new HashMap<>();
   private final HashMap<ConnectionPoint3D, ConnectionData> incomingBestEdge = new HashMap<>();

   private PriorityQueue<ConnectionPoint3D> stack;
   private ConnectionPoint3D closestPointToGoal;

   private void initialize(ConnectionPoint3D startPoint)
   {
      closestPointToGoal = startPoint;
      nodeCosts.put(startPoint, 0.0);
      stack = new PriorityQueue<>(new ConnectionPointComparator(nodeCosts));
      stack.add(startPoint);
   }

   private void buildVisibilityMap(Collection<VisibilityMapHolder> visibilityMapHolders)
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

   /**
    * Plans a path over the visibility maps
    * @return shortest path to the goal if one exists. Otherwise it will return a path to the
    * closest possible node to the goal
    */
   public List<Point3DReadOnly> calculatePath(ConnectionPoint3D startPoint, ConnectionPoint3D goalPoint, Collection<VisibilityMapHolder> visibilityMapHolders)
   {
      buildVisibilityMap(visibilityMapHolders);
      initialize(startPoint);

      stackLoop:
      while(!stack.isEmpty())
      {
         ConnectionPoint3D sourcePoint = stack.poll();

         HashSet<ConnectionData> connections = visibilityMap.computeIfAbsent(sourcePoint, (p) -> new HashSet<>());
         if(sourcePoint.distance(goalPoint) < closestPointToGoal.distance(goalPoint))
            closestPointToGoal = sourcePoint;

         for (ConnectionData connectionData : connections)
         {
            Connection connection = connectionData.connection;
            ConnectionPoint3D targetPoint = connection.getOppositePoint(sourcePoint);

            double nodeCost = nodeCosts.get(sourcePoint) + connectionData.edgeWeight;

            if(!nodeCosts.containsKey(targetPoint) || nodeCosts.get(targetPoint) > nodeCost)
            {
               nodeCosts.put(targetPoint, nodeCost);
               incomingBestEdge.put(targetPoint, connectionData);

               if(targetPoint.equals(goalPoint))
               {
                  break stackLoop;
               }
               else
               {
                  stack.add(targetPoint);
               }
            }
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

   private List<Point3DReadOnly> getPathToPoint(ConnectionPoint3D point)
   {
      List<Point3DReadOnly> path = new ArrayList<Point3DReadOnly>(){{add(point);}};

      ConnectionData incomingEdge = incomingBestEdge.get(point);
      ConnectionPoint3D previousTargetPoint = new ConnectionPoint3D(point);

      while(incomingEdge != null)
      {
         Connection connection = incomingEdge.connection;
         ConnectionPoint3D targetPoint = connection.getOppositePoint(previousTargetPoint);
         path.add(targetPoint);
         incomingEdge = incomingBestEdge.get(targetPoint);

         previousTargetPoint = targetPoint;
      }

      Collections.reverse(path);
      return path;
   }
}
