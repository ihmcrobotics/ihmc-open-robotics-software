package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class VisibilityGraphRandomTools
{
   public static Cluster getRandomCluster(Random random)
   {
      byte typeByte = (byte) RandomNumbers.nextInt(random, 0, Cluster.Type.values.length - 1);
      byte extrusionSideByte = (byte) RandomNumbers.nextInt(random, 0, Cluster.ExtrusionSide.values.length - 1);

      int numberOfRawPoints = RandomNumbers.nextInt(random, 1, 100);
      int numberOfNavigableExtrusions = RandomNumbers.nextInt(random, 1, 100);
      int numberOfNonNavigableExtrusions = RandomNumbers.nextInt(random, 1, 100);
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

   public static VisibilityMapWithNavigableRegion getRandomNavigableRegion(Random random)
   {
      int numberOfObstacleClusters = RandomNumbers.nextInt(random, 1, 50);
      PlanarRegion homeRegion = nextPlanarRegion(random);
      Cluster homeCluster = getRandomCluster(random);

      VisibilityMapWithNavigableRegion navigableRegion = new VisibilityMapWithNavigableRegion(homeRegion);
      navigableRegion.setHomeRegionCluster(homeCluster);
      for (int i = 0; i < numberOfObstacleClusters; i++)
         navigableRegion.addObstacleCluster(getRandomCluster(random));

      navigableRegion.setVisibilityMapInLocal(getRandomSingleSourceVisibilityMap(random).getVisibilityMapInLocal());

      return navigableRegion;
   }


   public static Connection getRandomConnection(Random random)
   {
      int startId = RandomNumbers.nextInt(random, 0, 100);
      int targetId = RandomNumbers.nextInt(random, 0, 100);
      Point3DReadOnly startPoint = EuclidCoreRandomTools.nextPoint3D(random, 50);
      Point3DReadOnly targetPoint = EuclidCoreRandomTools.nextPoint3D(random, 50);

      return new Connection(startPoint, startId, targetPoint, targetId);
   }

   public static VisibilityMapHolder getRandomSingleSourceVisibilityMap(Random random)
   {
      int numberOfConnections = RandomNumbers.nextInt(random, 2, 50);

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

      int numberOfConnections = RandomNumbers.nextInt(random, 2, 100);

      Set<Connection> connections = new HashSet<>();
      for (int i = 0; i < numberOfConnections; i++)
         connections.add((VisibilityGraphRandomTools.getRandomConnection(random)));
      map.addConnections(connections);
      map.getVisibilityMapInWorld().computeVertices();
      map.getVisibilityMapInLocal().computeVertices();

      return map;
   }

   public static VisibilityMap getRandomVisibilityMap(Random random)
   {
      int numberOfConnections = RandomNumbers.nextInt(random, 2, 100);

      Set<Connection> connections = new HashSet<>();
      for (int i = 0; i < numberOfConnections; i++)
         connections.add(VisibilityGraphRandomTools.getRandomConnection(random));

      VisibilityMap map = new VisibilityMap(connections);

      return map;
   }

   private static PlanarRegion nextPlanarRegion(Random random)
   {
      RigidBodyTransform transformToWorld = nextRegionTransform(random);
      Point2D[] concaveHullVertices = nextPoint2DArray(random);
      List<ConvexPolygon2D> convexPolygons = nextConvexPolygon2Ds(random);
      PlanarRegion next = new PlanarRegion(transformToWorld, concaveHullVertices, convexPolygons);
      next.setRegionId(random.nextInt());
      return next;
   }

   private static RigidBodyTransform nextRegionTransform(Random random)
   {
      Vector3D regionNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
      Point3D regionOrigin = EuclidCoreRandomTools.nextPoint3D(random);
      return new RigidBodyTransform(EuclidGeometryTools.axisAngleFromZUpToVector3D(regionNormal), regionOrigin);
   }

   private static List<ConvexPolygon2D> nextConvexPolygon2Ds(Random random)
   {
      int size = random.nextInt(100);
      return IntStream.range(0, size).mapToObj(i -> EuclidGeometryRandomTools.nextConvexPolygon2D(random, 10.0, 100)).collect(Collectors.toList());
   }

   private static Point2D[] nextPoint2DArray(Random random)
   {
      int size = random.nextInt(500);
      return IntStream.range(0, size).mapToObj(i -> EuclidCoreRandomTools.nextPoint2D(random)).toArray(Point2D[]::new);
   }
}
