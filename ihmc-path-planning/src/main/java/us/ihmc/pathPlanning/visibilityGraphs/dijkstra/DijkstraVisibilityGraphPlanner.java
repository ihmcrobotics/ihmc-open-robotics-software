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

            this.visibilityMap.putIfAbsent(connection.getSourcePoint(), new HashSet<>()).add(connectionData);
            this.visibilityMap.putIfAbsent(connection.getTargetPoint(), new HashSet<>()).add(connectionData);
         }
      }
   }

   public void initialize(ConnectionPoint3D startPoint, ConnectionPoint3D goalPoint)
   {
      this.startPoint = startPoint;
      this.goalPoint = goalPoint;

      nodeCosts.put(startPoint, 0.0);
      closestPointToGoal = null;
      stack = new PriorityQueue<>(new ConnectionPointComparator(nodeCosts));
      stack.add(startPoint);
   }

   /**
    * Plans a path over the visibility maps given by {@link #setVisibilityMap}
    * @param startPoint
    * @param goalPoint
    * @return if the planner succeeded in finding a complete path
    */
   public boolean plan(ConnectionPoint3D startPoint, ConnectionPoint3D goalPoint)
   {
      while(!stack.isEmpty())
      {
         ConnectionPoint3D sourcePoint = stack.poll();
         HashSet<ConnectionData> connections = visibilityMap.getOrDefault(sourcePoint, new HashSet<>());

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

      return nodeCosts.containsKey(goalPoint);
   }

   public List<Point3DReadOnly> getPath()
   {
      List<Point3DReadOnly> path = new ArrayList<>();
      path.add(goalPoint);

      Connection incomingEdge = incomingBestEdge.get(goalPoint).connection;
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
