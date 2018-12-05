package us.ihmc.pathPlanning.visibilityGraphs.dijkstra;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

public class DijkstraVisibilityGraphPlanner implements VisibilityGraphPathPlanner
{
   private final HashMap<ConnectionPoint3D, HashSet<ConnectionData>> visibilityMap = new HashMap<>();
   private final HashMap<ConnectionPoint3D, Double> nodeCosts = new HashMap<>();
   private final HashMap<ConnectionPoint3D, ConnectionData> incomingBestEdge = new HashMap<>();

   private PriorityQueue<ConnectionPoint3D> stack;

   private void initialize(ConnectionPoint3D startPoint)
   {
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

      ConnectionPoint3D closestPointToGoal = startPoint;
      double closestDistanceToGoalSquared = startPoint.distanceSquared(goalPoint);

      double bestCostToGoal = Double.POSITIVE_INFINITY;

      stackLoop: while (!stack.isEmpty())
      {
         ConnectionPoint3D sourcePoint = stack.poll();
         double sourceNodeCost = nodeCosts.get(sourcePoint);

         if (sourceNodeCost > bestCostToGoal)
            break stackLoop;

         HashSet<ConnectionData> connections = visibilityMap.computeIfAbsent(sourcePoint, (p) -> new HashSet<>());
         double distanceToGoalSquared = sourcePoint.distanceSquared(goalPoint);
         if (distanceToGoalSquared < closestDistanceToGoalSquared)
         {
            closestPointToGoal = sourcePoint;
            closestDistanceToGoalSquared = distanceToGoalSquared;
         }

         for (ConnectionData connectionData : connections)
         {
            Connection connection = connectionData.connection;
            ConnectionPoint3D targetPoint = connection.getOppositePoint(sourcePoint);

            double targetNodeCost = sourceNodeCost + connectionData.edgeWeight;

            if (!nodeCosts.containsKey(targetPoint) || nodeCosts.get(targetPoint) > targetNodeCost)
            {
               nodeCosts.put(targetPoint, targetNodeCost);
               incomingBestEdge.put(targetPoint, connectionData);

               if (targetPoint.equals(goalPoint))
               {
                  bestCostToGoal = targetNodeCost;
               }
               else
               {
                  stack.add(targetPoint);
               }
            }
         }
      }

      if (nodeCosts.containsKey(goalPoint))
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
      List<Point3DReadOnly> path = new ArrayList<Point3DReadOnly>();
      path.add(point);

      ConnectionData incomingEdge = incomingBestEdge.get(point);
      ConnectionPoint3D previousTargetPoint = new ConnectionPoint3D(point);

      while (incomingEdge != null)
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
