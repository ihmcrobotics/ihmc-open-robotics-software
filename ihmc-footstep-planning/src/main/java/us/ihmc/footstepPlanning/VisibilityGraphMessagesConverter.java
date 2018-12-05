package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

import java.util.ArrayList;
import java.util.List;

public class VisibilityGraphMessagesConverter
{
   public static BodyPathPlanStatisticsMessage convertToBodyPathPlanStatisticsMessage(VisibilityGraphStatistics statistics)
   {
      return convertToBodyPathPlanStatisticsMessage(BodyPathPlanStatisticsMessage.NO_PLAN_ID, statistics);
   }

   public static BodyPathPlanStatisticsMessage convertToBodyPathPlanStatisticsMessage(int planId, VisibilityGraphStatistics statistics)
   {
      BodyPathPlanStatisticsMessage message = new BodyPathPlanStatisticsMessage();

      message.setPlanId(planId);
      message.getGoalVisibilityMap().set(convertToVisibilityMapMessage(statistics.getGoalMapId(), statistics.getGoalVisibilityMap()));
      message.getStartVisibilityMap().set(convertToVisibilityMapMessage(statistics.getStartMapId(), statistics.getStartVisibilityMap()));
      message.getInterRegionsMap().set(convertToVisibilityMapMessage(statistics.getInterRegionsMapId(), statistics.getInterRegionsVisibilityMap()));

      for (int i = 0; i < statistics.getNumberOfNavigableRegions(); i++)
         message.getNavigableRegions().add().set(convertToNavigableRegionMessage(statistics.getNavigableRegion(i)));

      return message;
   }

   public static VisibilityMapMessage convertToVisibilityMapMessage(VisibilityMapHolder visibilityMapHolder)
   {
      return convertToVisibilityMapMessage(visibilityMapHolder.getMapId(), visibilityMapHolder.getVisibilityMapInWorld());
   }

   public static VisibilityMapMessage convertToVisibilityMapMessage(VisibilityMap visibilityMap)
   {
      return convertToVisibilityMapMessage(-1, visibilityMap);
   }

   public static VisibilityMapMessage convertToVisibilityMapMessage(int mapId, VisibilityMap visibilityMap)
   {
      VisibilityMapMessage message = new VisibilityMapMessage();

      if (visibilityMap == null)
         return message;

      for (Connection connection : visibilityMap.getConnections())
      {
         ConnectionPoint3D sourcePoint = connection.getSourcePoint();
         ConnectionPoint3D targetPoint = connection.getTargetPoint();
         message.getSourcePoints().add().set(sourcePoint);
         message.getTargetPoints().add().set(targetPoint);
         message.getSourceRegionIds().add(sourcePoint.getRegionId());
         message.getTargetRegionIds().add(targetPoint.getRegionId());
      }

      message.setMapId(mapId);

      return message;
   }

   public static NavigableRegionMessage convertToNavigableRegionMessage(NavigableRegion navigableRegion)
   {
      NavigableRegionMessage message = new NavigableRegionMessage();

      message.getHomeRegion().set(PlanarRegionMessageConverter.convertToPlanarRegionMessage(navigableRegion.getHomePlanarRegion()));
      message.getHomeRegionCluster().set(convertToVisibilityClusterMessage(navigableRegion.getHomeRegionCluster()));
      message.getVisibilityMapInWorld().set(convertToVisibilityMapMessage(navigableRegion.getMapId(), navigableRegion.getVisibilityMapInWorld()));

      List<Cluster> obstacleClusters = navigableRegion.getObstacleClusters();

      for (int i = 0; i < obstacleClusters.size(); i++)
         message.getObstacleClusters().add().set(convertToVisibilityClusterMessage(obstacleClusters.get(i)));

      return message;
   }

   public static VisibilityClusterMessage convertToVisibilityClusterMessage(Cluster cluster)
   {
      VisibilityClusterMessage message = new VisibilityClusterMessage();

      if (cluster == null)
         return message;

      List<? extends Point3DReadOnly> rawPointsInLocal = cluster.getRawPointsInLocal3D();
      List<Point2DReadOnly> navigableExtrusionsInLocal = cluster.getNavigableExtrusionsInLocal();
      List<Point2DReadOnly> nonNavigableExtrusionsInLocal = cluster.getNonNavigableExtrusionsInLocal();

      message.setExtrusionSide(cluster.getExtrusionSide().toByte());
      message.setType(cluster.getType().toByte());
      message.getPoseInWorld().set(cluster.getTransformToWorld());
      for (int i = 0; i < rawPointsInLocal.size(); i++)
         message.getRawPointsInLocal().add().set(rawPointsInLocal.get(i));
      for (int i = 0; i < navigableExtrusionsInLocal.size(); i++)
         message.getNavigableExtrusionsInLocal().add().set(navigableExtrusionsInLocal.get(i));
      for (int i = 0; i < nonNavigableExtrusionsInLocal.size(); i++)
         message.getNonNavigableExtrusionsInLocal().add().set(nonNavigableExtrusionsInLocal.get(i));

      return message;
   }

   public static VisibilityMapHolder convertToInterRegionsVisibilityMap(VisibilityMapMessage message)
   {
      InterRegionVisibilityMap visibilityMapHolder = new InterRegionVisibilityMap();

      for (int i = 0; i < message.getSourcePoints().size(); i++)
      {
         Point3D source = message.getSourcePoints().get(i);
         Point3D target = message.getTargetPoints().get(i);
         int sourceId = message.getSourceRegionIds().get(i);
         int targetId = message.getTargetRegionIds().get(i);
         visibilityMapHolder.addConnection(new Connection(source, sourceId, target, targetId));
      }
      visibilityMapHolder.getVisibilityMapInWorld().computeVertices();

      return visibilityMapHolder;
   }

   public static VisibilityMapHolder convertToSingleSourceVisibilityMap(VisibilityMapMessage message)
   {
      VisibilityMap visibilityMap = new VisibilityMap();

      for (int i = 0; i < message.getSourcePoints().size(); i++)
      {
         Point3D source = message.getSourcePoints().get(i);
         Point3D target = message.getTargetPoints().get(i);
         int sourceId = message.getSourceRegionIds().get(i);
         int targetId = message.getTargetRegionIds().get(i);
         visibilityMap.addConnection(new Connection(source, sourceId, target, targetId));
      }
      visibilityMap.computeVertices();
      VisibilityMapHolder mapHolder = new VisibilityMapHolder()
      {
         @Override
         public int getMapId()
         {
            return message.getMapId();
         }

         @Override
         public VisibilityMap getVisibilityMapInLocal()
         {
            throw new RuntimeException("This is not able to be returned as we have no knowledge of the local region.");
         }

         @Override
         public VisibilityMap getVisibilityMapInWorld()
         {
            return visibilityMap;
         }
      };

      return mapHolder;
   }

   public static List<NavigableRegion> convertToNavigableRegionsList(List<NavigableRegionMessage> message)
   {
      List<NavigableRegion> navigableRegionList = new ArrayList<>();

      for (int i = 0; i < message.size(); i++)
         navigableRegionList.add(convertToNavigableRegion(message.get(i)));

      return navigableRegionList;
   }

   public static NavigableRegion convertToNavigableRegion(NavigableRegionMessage message)
   {
      NavigableRegion navigableRegion = new NavigableRegion(PlanarRegionMessageConverter.convertToPlanarRegion(message.getHomeRegion()));

      navigableRegion.setHomeRegionCluster(convertToCluster(message.getHomeRegionCluster()));
      navigableRegion.setVisibilityMapInWorld(convertToVisibilityMap(message.getVisibilityMapInWorld()));

      List<VisibilityClusterMessage> obstacleClusterMessages = message.getObstacleClusters();
      for (int i = 0; i < obstacleClusterMessages.size(); i++)
         navigableRegion.addObstacleCluster(convertToCluster(obstacleClusterMessages.get(i)));

      return navigableRegion;
   }

   public static VisibilityMap convertToVisibilityMap(VisibilityMapMessage message)
   {
      VisibilityMap visibilityMap = new VisibilityMap();
      for (int i = 0; i < message.getSourcePoints().size(); i++)
      {
         visibilityMap.addConnection(
               new Connection(message.getSourcePoints().get(i), (int) message.getSourceRegionIds().get(i), message.getTargetPoints().get(i),
                              (int) message.getTargetRegionIds().get(i)));
      }
      visibilityMap.computeVertices();

      return visibilityMap;
   }

   public static Cluster convertToCluster(VisibilityClusterMessage message)
   {
      Cluster cluster = new Cluster();

      cluster.setType(Cluster.Type.fromByte(message.getType()));
      cluster.setExtrusionSide(Cluster.ExtrusionSide.fromByte(message.getExtrusionSide()));

      Pose3D poseInWorld = message.getPoseInWorld();
      RigidBodyTransform transform = new RigidBodyTransform();

      poseInWorld.get(transform);

      List<Point3D> rawPointsInLocal = message.getRawPointsInLocal();
      List<Point3D> navigableExtrusionsInLocal = message.getNavigableExtrusionsInLocal();
      List<Point3D> nonNavigableExtrusionsInLocal = message.getNonNavigableExtrusionsInLocal();

      cluster.setTransformToWorld(transform);
      cluster.addRawPointsInLocal3D(rawPointsInLocal);
      for (int i = 0; i < navigableExtrusionsInLocal.size(); i++)
         cluster.addNavigableExtrusionInLocal(navigableExtrusionsInLocal.get(i));
      for (int i = 0; i < nonNavigableExtrusionsInLocal.size(); i++)
         cluster.addNonNavigableExtrusionInLocal(nonNavigableExtrusionsInLocal.get(i));

      return cluster;
   }

   public static VisibilityGraphStatistics convertToVisibilityGraphStatistics(BodyPathPlanStatisticsMessage message)
   {
      VisibilityGraphStatistics statistics = new VisibilityGraphStatistics();

      VisibilityMapHolder startMap = convertToSingleSourceVisibilityMap(message.getStartVisibilityMap());
      VisibilityMapHolder goalMap = convertToSingleSourceVisibilityMap(message.getGoalVisibilityMap());
      VisibilityMapHolder interRegionsMap = convertToInterRegionsVisibilityMap(message.getInterRegionsMap());

      statistics.setStartVisibilityMapInWorld(startMap.getMapId(), startMap.getVisibilityMapInWorld());
      statistics.setGoalVisibilityMapInWorld(goalMap.getMapId(), goalMap.getVisibilityMapInWorld());
      statistics.setInterRegionsVisibilityMapInWorld(interRegionsMap.getMapId(), interRegionsMap.getVisibilityMapInWorld());
      List<NavigableRegionMessage> navigableRegions = message.getNavigableRegions();
      for (int i = 0; i < navigableRegions.size(); i++)
         statistics.addNavigableRegion(convertToNavigableRegion(navigableRegions.get(i)));

      return statistics;
   }
}
