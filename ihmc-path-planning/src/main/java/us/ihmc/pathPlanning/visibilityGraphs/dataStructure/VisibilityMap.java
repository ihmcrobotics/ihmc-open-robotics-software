package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;

public class VisibilityMap implements Transformable, Iterable<Connection>
{
   private Set<Connection> connections;
   private Set<ConnectionPoint3D> vertices;

   public VisibilityMap()
   {
      connections = new HashSet<>();
   }

   public VisibilityMap(Set<Connection> connections)
   {
      this.connections = connections;
   }

   public VisibilityMap(VisibilityMap other)
   {
      this.connections = new HashSet<>(other.connections);
      computeVertices();
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

   public Set<ConnectionPoint3D> getVertices()
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
   public Iterator<Connection> iterator()
   {
      return connections.iterator();
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
