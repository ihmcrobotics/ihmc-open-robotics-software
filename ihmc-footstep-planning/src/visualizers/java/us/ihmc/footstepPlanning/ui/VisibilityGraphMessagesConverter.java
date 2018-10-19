package us.ihmc.footstepPlanning.ui;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

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

   public static VisibilityMapMessage convertToVisibilityMapMessage(int mapId, VisibilityMap visibilityMap)
   {
      VisibilityMapMessage message = new VisibilityMapMessage();

      for (Connection connection : visibilityMap.getConnections())
      {
         message.getSourcePoints().add().set(connection.getSourcePoint());
         message.getTargetPoints().add().set(connection.getTargetPoint());
         message.getSourceRegionIds().add(connection.getSourcePoint().getRegionId());
         message.getTargetRegionIds().add(connection.getTargetPoint().getRegionId());
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
      message.getPoseInWorld().set(navigableRegion.getTransformToWorld());

      List<Cluster> obstacleClusters = navigableRegion.getObstacleClusters();
      List<Cluster> allClusters = navigableRegion.getAllClusters();

      for (int i = 0; i < obstacleClusters.size(); i++)
         message.getObstacleClusters().add().set(convertToVisibilityClusterMessage(obstacleClusters.get(i)));
      for (int i = 0; i < allClusters.size(); i++)
      {
         Cluster otherCluster = allClusters.get(i);
         if (obstacleClusters.contains(otherCluster))
            continue;
         else
            message.getNonObstacleClusters().add().set(convertToVisibilityClusterMessage(otherCluster));
      }

      return message;
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

   public static VisibilityMapHolder convertToInterRegionsVisibilityMap(VisibilityMapMessage message)
   {
      InterRegionVisibilityMap visibilityMapHolder = new InterRegionVisibilityMap();

      for (int i = 0; i < message.getSourcePoints().size(); i++)
         visibilityMapHolder.addConnection(
               new Connection(message.getSourcePoints().get(i), (int) message.getSourceRegionIds().get(i), message.getTargetPoints().get(i),
                              (int) message.getTargetRegionIds().get(i)));

      return visibilityMapHolder;
   }

   public static VisibilityMapHolder convertToSingleSourceVisibilityMap(VisibilityMapMessage message)
   {
      VisibilityMap visibilityMap = new VisibilityMap();

      for (int i = 0; i < message.getSourcePoints().size(); i++)
         visibilityMap.addConnection(
               new Connection(message.getSourcePoints().get(i), (int) message.getSourceRegionIds().get(i), message.getTargetPoints().get(i),
                              (int) message.getTargetRegionIds().get(i)));
      VisibilityMapHolder mapHolder = new VisibilityMapHolder()
      {
         @Override
         public int getMapId()
         {
            return (int) message.getMapId();
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
      navigableRegion.setVisibilityMapInWorld(convertToSingleSourceVisibilityMap(message.getVisibilityMapInWorld()).getVisibilityMapInWorld());

      List<VisibilityClusterMessage> obstacleClusterMessages = message.getObstacleClusters();
      for (int i = 0; i < obstacleClusterMessages.size(); i++)
         navigableRegion.addObstacleCluster(convertToCluster(obstacleClusterMessages.get(i)));

      return navigableRegion;
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
      cluster.addNavigableExtrusionsInLocal3D(navigableExtrusionsInLocal);
      cluster.addNonNavigableExtrusionsInLocal3D(nonNavigableExtrusionsInLocal);

      return cluster;
   }
}
