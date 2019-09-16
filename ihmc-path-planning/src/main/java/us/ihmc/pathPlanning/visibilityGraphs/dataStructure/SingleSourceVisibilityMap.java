package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.Collection;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

public class SingleSourceVisibilityMap implements VisibilityMapHolder
{
   private final Point3D sourceInWorld, sourceInLocal;
   private final int mapId;
   private final RigidBodyTransformReadOnly transformToWorld;
   private final VisibilityMapWithNavigableRegion hostRegion;
   private final VisibilityMap visibilityMapInLocal;
   private final VisibilityMap visibilityMapInWorld;

   public SingleSourceVisibilityMap(Point3DReadOnly sourceInWorld, Collection<Connection> connectionsInLocal, VisibilityMapWithNavigableRegion hostRegion)
   {
      this.hostRegion = hostRegion;
      this.sourceInWorld = new Point3D(sourceInWorld);
      this.mapId = hostRegion.getMapId();
      this.transformToWorld = hostRegion.getTransformToWorld();

      sourceInLocal = new Point3D(sourceInWorld);
      hostRegion.transformFromWorldToLocal(sourceInLocal);

      visibilityMapInLocal = new VisibilityMap(connectionsInLocal);
      visibilityMapInWorld = new VisibilityMap(visibilityMapInLocal.getConnections());
      visibilityMapInWorld.applyTransform(transformToWorld);
      visibilityMapInWorld.computeVertices();
   }

   public SingleSourceVisibilityMap(Point3DReadOnly source, int mapId, Collection<Connection> connections)
   {
      sourceInLocal = new Point3D(source);
      sourceInWorld = sourceInLocal;
      this.mapId = mapId;

      visibilityMapInLocal = new VisibilityMap(connections);
      visibilityMapInWorld = visibilityMapInLocal;

      hostRegion = null;
      transformToWorld = new RigidBodyTransform();
   }

   public void addConnectionInWorld(Connection connectionInWorld)
   {
      visibilityMapInWorld.addConnection(connectionInWorld);
      visibilityMapInWorld.computeVertices();
      Connection connectionInLocal = new Connection(connectionInWorld);
      hostRegion.transformFromWorldToLocal(connectionInLocal);
      visibilityMapInLocal.addConnection(connectionInLocal);
      visibilityMapInLocal.computeVertices();
   }

   public Point3DReadOnly getSourceInWorld()
   {
      return sourceInWorld;
   }

   public Point3D getSourceInLocal3D()
   {
      return sourceInLocal;
   }

   public Point2D getSourceInLocal2D()
   {
      return new Point2D(sourceInLocal);
   }

   public VisibilityMapWithNavigableRegion getHostRegion()
   {
      return hostRegion;
   }

   @Override
   public int getMapId()
   {
      return mapId;
   }

   @Override
   public VisibilityMap getVisibilityMapInLocal()
   {
      return visibilityMapInLocal;
   }

   @Override
   public VisibilityMap getVisibilityMapInWorld()
   {
      return visibilityMapInWorld;
   }
}
