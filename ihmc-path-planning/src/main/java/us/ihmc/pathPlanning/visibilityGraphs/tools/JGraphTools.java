package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.jgrapht.Graph;
import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

public class JGraphTools
{
   public static List<Point3DReadOnly> calculatePathOnVisibilityGraph(ConnectionPoint3D start, ConnectionPoint3D goal, Collection<Connection> interConnections,
                                                                      Collection<VisibilityMapHolder> allVisibilityMapHolders)
   {
      SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graph = createGlobalVisibilityGraph(interConnections, allVisibilityMapHolders);
      List<DefaultWeightedEdge> solution = DijkstraShortestPath.findPathBetween(graph, start, goal);
      return convertVisibilityGraphSolutionToPath(solution, start, graph);
   }

   public static List<Point3DReadOnly> convertVisibilityGraphSolutionToPath(List<DefaultWeightedEdge> solution, Point3DReadOnly start,
                                                                            Graph<ConnectionPoint3D, DefaultWeightedEdge> graph)
   {
      if (solution == null)
         return null;

      List<Point3DReadOnly> path = new ArrayList<>();
      for (DefaultWeightedEdge edge : solution)
      {
         Point3DReadOnly from = graph.getEdgeSource(edge);
         Point3DReadOnly to = graph.getEdgeTarget(edge);

         if (!path.contains(new Point3D(from)))
            path.add(new Point3D(from));
         if (!path.contains(new Point3D(to)))
            path.add(new Point3D(to));
      }

      // FIXME Sylvain: it looks like this is to cover a bug.
      if (!path.get(0).epsilonEquals(start, 1e-5))
      {
         Point3DReadOnly pointOut = path.get(1);
         path.remove(1);
         path.add(0, pointOut);
      }

      return path;
   }

   public static void addConnectionToGraph(Connection connection, SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graphToUpdate)
   {
      ConnectionPoint3D source = connection.getSourcePoint();
      ConnectionPoint3D target = connection.getTargetPoint();

      if (!source.epsilonEquals(target, 1.0e-3))
      {
         graphToUpdate.addVertex(source);
         graphToUpdate.addVertex(target);
         DefaultWeightedEdge edge = new DefaultWeightedEdge();
         graphToUpdate.addEdge(source, target, edge);
         graphToUpdate.setEdgeWeight(edge, source.distance(target));
      }
   }

   public static void addConnectionsToGraph(Iterable<Connection> connections, SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graphToUpdate)
   {
      connections.forEach(connection -> addConnectionToGraph(connection, graphToUpdate));
   }

   public static SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> createGlobalVisibilityGraph(Collection<Connection> interConnections,
                                                                                                         Collection<VisibilityMapHolder> allVisibilityMapHolders)
   {
      SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

      addConnectionsToGraph(interConnections, globalVisMap);
      allVisibilityMapHolders.stream().map(VisibilityMapHolder::getVisibilityMapInWorld).map(VisibilityMap::getConnections)
                             .forEach(connections -> addConnectionsToGraph(connections, globalVisMap));

      return globalVisMap;
   }
}
