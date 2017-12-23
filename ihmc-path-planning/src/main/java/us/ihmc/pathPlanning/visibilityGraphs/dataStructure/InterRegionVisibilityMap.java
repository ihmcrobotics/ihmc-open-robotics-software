package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

public class InterRegionVisibilityMap implements VisibilityMapHolder
{
   private final VisibilityMap visibilityMap = new VisibilityMap();

   public InterRegionVisibilityMap()
   {
   }

   public void addCoonnections(Iterable<Connection> connections)
   {
      connections.forEach(this::addConnection);
   }

   public void addConnection(Connection connection)
   {
      visibilityMap.addConnection(connection);
   }

   public void addConnection(ConnectionPoint3D source, ConnectionPoint3D target)
   {
      visibilityMap.addConnection(new Connection(source, target));
   }

   @Override
   public int getMapId()
   {
      return 0;
   }

   @Override
   public VisibilityMap getVisibilityMapInLocal()
   {
      return visibilityMap;
   }

   @Override
   public VisibilityMap getVisibilityMapInWorld()
   {
      return visibilityMap;
   }
}
