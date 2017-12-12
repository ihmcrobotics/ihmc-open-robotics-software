package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.HashSet;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class VisibilityMap
{
   private HashSet<Connection> connections;
   private HashSet<Point3DReadOnly> vertices;

   public VisibilityMap()
   {
      connections = new HashSet<>();
   }

   public VisibilityMap(HashSet<Connection> connections)
   {
      this.connections = connections;
   }
   
   public void setConnections(HashSet<Connection> connections)
   {
      this.connections = connections;
   }
   
   public void addConnection(Connection connection)
   {
      connections.add(connection);
   }
   
   public void addConnections(HashSet<Connection> connections)
   {
      this.connections.addAll(connections);
   }
   
   
   public void computeVertices()
   {
      vertices = new HashSet<>();
      for (Connection connection : connections)
      {
         vertices.add(connection.getSourcePoint());
         vertices.add(connection.getTargetPoint());
      }
   }
   
   public HashSet<Point3DReadOnly> getVertices()
   {
      return vertices;
   }
   
   public HashSet<Connection> getConnections()
   {
      return connections;
   }
}
