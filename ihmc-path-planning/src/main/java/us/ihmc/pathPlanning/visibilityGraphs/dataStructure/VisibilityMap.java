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
   private final HashSet<ConnectionPoint3D> vertices;

   public VisibilityMap()
   {
      connections = new HashSet<>();
      vertices = new HashSet<>();
   }

   public VisibilityMap(Collection<Connection> connections)
   {
      this();
      setConnections(connections);
      computeVertices();
   }

   public VisibilityMap(VisibilityMap other)
   {
      this();
      set(other);
   }

   public void set(VisibilityMap other)
   {
      setConnections(other.connections);
      computeVertices();
   }

   public void copy(VisibilityMap other)
   {
      copyConnections(other.connections);
      computeVertices();
   }

   public void copyConnections(Collection<Connection> connections)
   {
      this.connections.clear();
      for (Connection connection : connections)
         this.connections.add(connection.getCopy());
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
      vertices.clear();
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
