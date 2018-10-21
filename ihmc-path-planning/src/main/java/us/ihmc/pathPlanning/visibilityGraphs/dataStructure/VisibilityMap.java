package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.*;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;

public class VisibilityMap implements Transformable, Iterable<Connection>
{
   private final List<Connection> connections;
   private final List<ConnectionPoint3D> vertices;

   public VisibilityMap()
   {
      connections = new ArrayList<>();
      vertices = new ArrayList<>();
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

   public void setConnections(Collection<Connection> connections)
   {
      this.connections.clear();
      this.connections.addAll(connections);
   }

   public void setConnections(Set<Connection> connections)
   {
      this.connections.clear();
      this.connections.addAll(connections);
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

   public List<ConnectionPoint3D> getVertices()
   {
      return vertices;
   }

   public List<Connection> getConnections()
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
