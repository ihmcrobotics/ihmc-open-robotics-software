package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

public class InterRegionVisibilityMap implements VisibilityMapHolder
{
   /**
    * This scale factor is a multiplier on inter-region connection weight. If this value greater
    * than 1.0, it will encourage paths with short inter-region connections. The benefit is discouraging
    * the use of long inter-region connections, which the planner favors when this scale is 1.0
    */
   private static final double INTER_REGION_WEIGHT_SCALE = 1.25;

   private final VisibilityMap visibilityMap = new VisibilityMap();

   public void addConnections(Iterable<Connection> connections)
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
   public double getConnectionWeight(Connection connection)
   {
      ConnectionPoint3D sourcePoint = connection.getSourcePoint();
      ConnectionPoint3D targetPoint = connection.getTargetPoint();

      double horizontalDistance = EuclidGeometryTools.distanceBetweenPoint2Ds(sourcePoint.getX(), sourcePoint.getY(), targetPoint.getX(), targetPoint.getY());
      double verticalDistance = Math.abs(sourcePoint.getZ() - targetPoint.getZ());

      return INTER_REGION_WEIGHT_SCALE * (horizontalDistance + verticalDistance);
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
