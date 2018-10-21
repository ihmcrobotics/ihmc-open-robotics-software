package us.ihmc.footstepPlanning;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.*;

public class VisibilityGraphRandomTools
{
   public static Cluster getRandomCluster(Random random)
   {
      byte typeByte = (byte) RandomNumbers.nextInt(random, 0, Cluster.Type.values.length - 1);
      byte extrusionSideByte = (byte) RandomNumbers.nextInt(random, 0, Cluster.ExtrusionSide.values.length - 1);

      int numberOfRawPoints = RandomNumbers.nextInt(random, 1, 1000);
      int numberOfNavigableExtrusions = RandomNumbers.nextInt(random, 1, 1000);
      int numberOfNonNavigableExtrusions = RandomNumbers.nextInt(random, 1, 1000);
      RigidBodyTransform transformToWorld = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      List<Point3D> rawPointsInLocalExpected = new ArrayList<>();
      List<Point2D> navigableExtrusionsInLocalExpected = new ArrayList<>();
      List<Point2D> nonNavigableExtrusionsInLocalExpected = new ArrayList<>();

      for (int i = 0; i < numberOfRawPoints; i++)
         rawPointsInLocalExpected.add(EuclidCoreRandomTools.nextPoint3D(random, 100.0));
      for (int i = 0; i < numberOfNavigableExtrusions; i++)
         navigableExtrusionsInLocalExpected.add(EuclidCoreRandomTools.nextPoint2D(random, 100.0));
      for (int i = 0; i < numberOfNonNavigableExtrusions; i++)
         nonNavigableExtrusionsInLocalExpected.add(EuclidCoreRandomTools.nextPoint2D(random, 100.0));

      Cluster cluster = new Cluster();
      cluster.setTransformToWorld(transformToWorld);
      cluster.setType(Cluster.Type.fromByte(typeByte));
      cluster.setExtrusionSide(Cluster.ExtrusionSide.fromByte(extrusionSideByte));

      for (int i = 0; i < numberOfRawPoints; i++)
         cluster.addRawPointInLocal(rawPointsInLocalExpected.get(i));
      for (int i = 0; i < numberOfNavigableExtrusions; i++)
         cluster.addNavigableExtrusionInLocal(navigableExtrusionsInLocalExpected.get(i));
      for (int i = 0; i < numberOfNonNavigableExtrusions; i++)
         cluster.addNonNavigableExtrusionInLocal(nonNavigableExtrusionsInLocalExpected.get(i));

      return cluster;
   }

   public static NavigableRegion getRandomNavigableRegion(Random random)
   {
      int numberOfPolygons = RandomNumbers.nextInt(random, 1, 100);
      int numberOfPoints = RandomNumbers.nextInt(random, 1, 100);
      int numberOfObstacleClusters = RandomNumbers.nextInt(random, 1, 100);
      PlanarRegion homeRegion = PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, numberOfPolygons, 1000.0, numberOfPoints);
      Cluster homeCluster = getRandomCluster(random);

      NavigableRegion navigableRegion = new NavigableRegion(homeRegion);
      navigableRegion.setHomeRegionCluster(homeCluster);
      for (int i = 0; i < numberOfObstacleClusters; i++)
         navigableRegion.addObstacleCluster(getRandomCluster(random));
      navigableRegion.setVisibilityMapInLocal(getRandomVisibilityMap(random));

      return navigableRegion;
   }

   public static VisibilityMap getRandomVisibilityMap(Random random)
   {
      VisibilityMap visibilityMap = new VisibilityMap();
      int numberOfConnections = RandomNumbers.nextInt(random, 2, 10000);

      for (int i = 0; i < numberOfConnections; i++)
         visibilityMap.addConnection(getRandomConnection(random));

      visibilityMap.computeVertices();

      return visibilityMap;
   }

   public static Connection getRandomConnection(Random random)
   {
      int startId = RandomNumbers.nextInt(random, 0, 1000);
      int targetId = RandomNumbers.nextInt(random, 0, 1000);
      Point3DReadOnly startPoint = EuclidCoreRandomTools.nextPoint3D(random, 100);
      Point3DReadOnly targetPoint = EuclidCoreRandomTools.nextPoint3D(random, 100);

      return new Connection(startPoint, startId, targetPoint, targetId);
   }

   public static VisibilityMapHolder getRandomSingleSourceVisibilityMap(Random random)
   {
      int numberOfConnections = RandomNumbers.nextInt(random, 2, 10000);

      Set<Connection> connections = new HashSet<>();
      for (int i = 0; i < numberOfConnections; i++)
         connections.add(VisibilityGraphRandomTools.getRandomConnection(random));

      VisibilityMapHolder map =  new SingleSourceVisibilityMap(EuclidCoreRandomTools.nextPoint3D(random, 100.0), RandomNumbers.nextInt(random, -100, 100), connections);
      map.getVisibilityMapInLocal().computeVertices();
      map.getVisibilityMapInWorld().computeVertices();

      return map;
   }

   public static VisibilityMapHolder getRandomInterRegionVisibilityMap(Random random)
   {
      InterRegionVisibilityMap map = new InterRegionVisibilityMap();

      int numberOfConnections = RandomNumbers.nextInt(random, 2, 10000);

      List<Connection> connections = new ArrayList<>();
      for (int i = 0; i < numberOfConnections; i++)
         connections.add((VisibilityGraphRandomTools.getRandomConnection(random)));
      map.addConnections(connections);
      map.getVisibilityMapInWorld().computeVertices();
      map.getVisibilityMapInLocal().computeVertices();

      return map;
   }
}
