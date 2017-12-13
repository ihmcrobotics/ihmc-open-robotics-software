package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class VisibilityMap implements Transformable
{
   private Set<Connection> connections;
   private Set<Point3DReadOnly> vertices;

   public VisibilityMap()
   {
      connections = new HashSet<>();
   }

   public VisibilityMap(HashSet<Connection> connections)
   {
      this.connections = connections;
   }

   public void setConnections(Collection<Connection> connections)
   {
      this.connections = new HashSet<>(connections);
   }

   public void setConnections(Set<Connection> connections)
   {
      this.connections = connections;
   }

   public void addConnection(Connection connection)
   {
      connections.add(connection);
   }

   public void addConnections(Set<Connection> connections)
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

   public Set<Point3DReadOnly> getVertices()
   {
      return vertices;
   }

   public Set<Connection> getConnections()
   {
      return connections;
   }

   public boolean isEmpty()
   {
      return connections.isEmpty();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      connections.forEach(c -> c.applyTransform(transform));
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      connections.forEach(c -> c.applyInverseTransform(transform));
   }
}
