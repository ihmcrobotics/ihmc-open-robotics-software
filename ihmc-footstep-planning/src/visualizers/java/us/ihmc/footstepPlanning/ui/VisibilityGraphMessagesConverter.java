package us.ihmc.footstepPlanning.ui;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

import java.util.List;
import java.util.Set;

public class VisibilityGraphMessagesConverter
{
   public static BodyPathPlanStatisticsMessage convertToBodyPathPlanStatisticsMessage(VisibilityGraphStatistics statistics)
   {
      BodyPathPlanStatisticsMessage message = new BodyPathPlanStatisticsMessage();

      throw new RuntimeException("Not Yet Implemented.");
   }

   public static VisibilityMapMessage convertToVisibilityMapMessage(VisibilityMapHolder visibilityMapHolder)
   {
      return convertToVisibilityMapMessage(visibilityMapHolder.getMapId(), visibilityMapHolder.getVisibilityMapInWorld());
   }

   public static VisibilityMapMessage convertToVisibilityMapMessage(int mapId, VisibilityMap visibilityMap)
   {
      VisibilityMapMessage message = new VisibilityMapMessage();

      for (Connection connection : visibilityMap.getConnections())
      {
         message.getSourcePoints().add().set(connection.getSourcePoint());
         message.getTargetPoints().add().set(connection.getTargetPoint());
      }

      message.setMapId(mapId);

      return message;
   }

   public static NavigableRegionMessage convertToNavigableRegionMessage(NavigableRegion navigableRegion)
   {
      NavigableRegionMessage message = new NavigableRegionMessage();

      message.getHomeRegion().set(PlanarRegionMessageConverter.convertToPlanarRegionMessage(navigableRegion.getHomeRegion()));
      message.getHomeRegionCluster().set(convertToVisibilityClusterMessage(navigableRegion.getHomeRegionCluster()));
      message.getVisibilityMapInWorld().set(convertToVisibilityMapMessage(navigableRegion.getMapId(), navigableRegion.getVisibilityMapInWorld()));

      List<Cluster> obstacleClusters = navigableRegion.getObstacleClusters();
      List<Cluster> allClusters = navigableRegion.getAllClusters();

      for (int i = 0; i < obstacleClusters.size(); i++)
         message.getObstacleClusters().add().set(convertToVisibilityClusterMessage(obstacleClusters.get(i)));

      throw new RuntimeException("Not Yet Implemented.");

   }

   public static VisibilityClusterMessage convertToVisibilityClusterMessage(Cluster cluster)
   {
      VisibilityClusterMessage message = new VisibilityClusterMessage();

      List<Point3D> rawPointsInLocal = cluster.getRawPointsInLocal3D();
      List<Point2D> navigableExtrusionsInLocal = cluster.getNavigableExtrusionsInLocal2D();
      List<Point2D> nonNavigableExtrusionsInLocal = cluster.getNonNavigableExtrusionsInLocal2D();

      message.setExtrusionSide(cluster.getExtrusionSide().toByte());
      message.setType(cluster.getType().toByte());
      message.getPoseInWorld().set(cluster.getTransformToWorld());
      for (int i = 0; i < rawPointsInLocal.size(); i++)
         message.getRawPointsInLocal().add().set(rawPointsInLocal.get(i));
      for (int i = 0; i < navigableExtrusionsInLocal.size(); i++)
         message.getNavigableExtrusionsInLocal().add().set(navigableExtrusionsInLocal.get(i));
      for (int i = 0; i < rawPointsInLocal.size(); i++)
         message.getNonNavigableExtrusionsInLocal().add().set(nonNavigableExtrusionsInLocal.get(i));

      return message;
   }

   public static VisibilityMapHolder convertToVisibilityMapHolder(VisibilityMapMessage message)
   {
      throw new RuntimeException("Not Yet Implemented.");
   }

   public static List<NavigableRegion> convertToNavigableRegionsList(List<NavigableRegionMessage> message)
   {
      throw new RuntimeException("Not Yet Implemented.");
   }
}
