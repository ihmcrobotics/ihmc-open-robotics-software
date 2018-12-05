package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.*;
import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphRandomTools;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphTestTools;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.SingleSourceVisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;

public class VisibilityGraphMessagesConverterTest
{
   private static final int iters = 1000;
   private static final double epsilon = 1e-9;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration =  1.0)
   @Test(timeout = 30000)
   public void testConvertToCluster()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
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

         Cluster clusterToConvert = new Cluster();
         clusterToConvert.setTransformToWorld(transformToWorld);
         clusterToConvert.setType(Cluster.Type.fromByte(typeByte));
         clusterToConvert.setExtrusionSide(Cluster.ExtrusionSide.fromByte(extrusionSideByte));

         for (int i = 0; i < numberOfRawPoints; i++)
            clusterToConvert.addRawPointInLocal(rawPointsInLocalExpected.get(i));
         for (int i = 0; i < numberOfNavigableExtrusions; i++)
            clusterToConvert.addNavigableExtrusionInLocal(navigableExtrusionsInLocalExpected.get(i));
         for (int i = 0; i < numberOfNonNavigableExtrusions; i++)
            clusterToConvert.addNonNavigableExtrusionInLocal(nonNavigableExtrusionsInLocalExpected.get(i));

         VisibilityClusterMessage message = VisibilityGraphMessagesConverter.convertToVisibilityClusterMessage(clusterToConvert);
         Cluster convertedCluster = VisibilityGraphMessagesConverter.convertToCluster(message);

         EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(transformToWorld, convertedCluster.getTransformToWorld(), epsilon);
         EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(transformToWorld, clusterToConvert.getTransformToWorld(), epsilon);
         assertEquals(numberOfRawPoints, convertedCluster.getNumberOfRawPoints());
         assertEquals(numberOfRawPoints, clusterToConvert.getNumberOfRawPoints());
         assertEquals(numberOfNavigableExtrusions, convertedCluster.getNumberOfNavigableExtrusions());
         assertEquals(numberOfNavigableExtrusions, clusterToConvert.getNumberOfNavigableExtrusions());
         assertEquals(numberOfNonNavigableExtrusions, convertedCluster.getNumberOfNonNavigableExtrusions());
         assertEquals(numberOfNonNavigableExtrusions, clusterToConvert.getNumberOfNonNavigableExtrusions());

         ReferenceFrame localFrame = new ReferenceFrame("localFrame", worldFrame)
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               transformToParent.set(transformToWorld);
            }
         };
         localFrame.update();

         for (int i = 0; i < numberOfRawPoints; i++)
         {
            Point3D expectedPoint = new Point3D(rawPointsInLocalExpected.get(i));
            FramePoint3D expectedFramePoint = new FramePoint3D(localFrame, rawPointsInLocalExpected.get(i));
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedPoint, clusterToConvert.getRawPointInLocal(i), epsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedPoint, convertedCluster.getRawPointInLocal(i), epsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedFramePoint, clusterToConvert.getRawPointInLocal(i), epsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedFramePoint, convertedCluster.getRawPointInLocal(i), epsilon);
            transformToWorld.transform(expectedPoint);
            expectedFramePoint.changeFrame(worldFrame);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedPoint, clusterToConvert.getRawPointInWorld(i), epsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedPoint, clusterToConvert.getRawPointInWorld(i), epsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedFramePoint, clusterToConvert.getRawPointInWorld(i), epsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedFramePoint, convertedCluster.getRawPointInWorld(i), epsilon);
         }
         for (int i = 0; i < numberOfNavigableExtrusions; i++)
         {
            EuclidCoreTestTools
                  .assertPoint2DGeometricallyEquals(navigableExtrusionsInLocalExpected.get(i), clusterToConvert.getNavigableExtrusionInLocal(i), epsilon);
            EuclidCoreTestTools
                  .assertPoint2DGeometricallyEquals(navigableExtrusionsInLocalExpected.get(i), convertedCluster.getNavigableExtrusionInLocal(i), epsilon);
         }
         for (int i = 0; i < numberOfNonNavigableExtrusions; i++)
         {
            EuclidCoreTestTools
                  .assertPoint2DGeometricallyEquals(nonNavigableExtrusionsInLocalExpected.get(i), clusterToConvert.getNonNavigableExtrusionInLocal(i), epsilon);
            EuclidCoreTestTools
                  .assertPoint2DGeometricallyEquals(nonNavigableExtrusionsInLocalExpected.get(i), convertedCluster.getNonNavigableExtrusionInLocal(i), epsilon);
         }

         VisibilityGraphTestTools.assertClustersEqual(clusterToConvert, convertedCluster, epsilon);
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration =  2.6)
   @Test(timeout = 30000)
   public void testConvertInterRegionsVisibilityMap()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         VisibilityMapHolder mapToConvert = VisibilityGraphRandomTools.getRandomInterRegionVisibilityMap(random);

         VisibilityMapMessage visibilityMapMessage = VisibilityGraphMessagesConverter.convertToVisibilityMapMessage(mapToConvert);
         VisibilityMapHolder convertedMap = VisibilityGraphMessagesConverter.convertToInterRegionsVisibilityMap(visibilityMapMessage);

         VisibilityGraphTestTools.assertVisibilityMapHoldersEqual(mapToConvert, convertedMap, epsilon);
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration =  1.8)
   @Test(timeout = 30000)
   public void testConvertSingleSourceVisibilityMap()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         int numberOfConnections = RandomNumbers.nextInt(random, 2, 100);

         List<Connection> connections = new ArrayList<>();
         for (int i = 0; i < numberOfConnections; i++)
            connections.add(VisibilityGraphRandomTools.getRandomConnection(random));

         VisibilityMapHolder mapToConvert = new SingleSourceVisibilityMap(EuclidCoreRandomTools.nextPoint3D(random, 100.0),
                                                                          RandomNumbers.nextInt(random, -100, 100), connections);

         VisibilityMapMessage visibilityMapMessage = VisibilityGraphMessagesConverter.convertToVisibilityMapMessage(mapToConvert);
         VisibilityMapHolder convertedMap = VisibilityGraphMessagesConverter.convertToSingleSourceVisibilityMap(visibilityMapMessage);

         VisibilityGraphTestTools.assertSingleSourceVisibilityMapHoldersEqual(mapToConvert, convertedMap, epsilon);
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration =  14.8)
   @Test(timeout = 30000)
   public void testConvertNavigableRegion()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         VisibilityMapWithNavigableRegion navigableRegionToConvert = VisibilityGraphRandomTools.getRandomNavigableRegion(random);
         NavigableRegionMessage message = VisibilityGraphMessagesConverter.convertToNavigableRegionMessage(navigableRegionToConvert);

         VisibilityMapWithNavigableRegion convertedNavigableRegion = VisibilityGraphMessagesConverter.convertToNavigableRegion(message);

         VisibilityGraphTestTools.assertNavigableRegionsEqual(navigableRegionToConvert, convertedNavigableRegion, epsilon);
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration =  1.8)
   @Test(timeout = 30000)
   public void testConvertVisibilityMap()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         VisibilityMap mapToConvert = VisibilityGraphRandomTools.getRandomVisibilityMap(random);
         VisibilityMapMessage message = VisibilityGraphMessagesConverter.convertToVisibilityMapMessage(mapToConvert);
         VisibilityMap convertedMap = VisibilityGraphMessagesConverter.convertToVisibilityMap(message);

         VisibilityGraphTestTools.assertVisibilityMapsEqual(mapToConvert, convertedMap, epsilon);
      }
   }


   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration =  135.9)
   @Test(timeout = 600000)
   public void testConvertBodyPathPlanStatistics()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         VisibilityMapHolder startMap = VisibilityGraphRandomTools.getRandomSingleSourceVisibilityMap(random);
         VisibilityMapHolder goalMap = VisibilityGraphRandomTools.getRandomSingleSourceVisibilityMap(random);
         VisibilityMapHolder interRegionsMap = VisibilityGraphRandomTools.getRandomInterRegionVisibilityMap(random);

         List<VisibilityMapWithNavigableRegion> navigableRegions = new ArrayList<>();
         int planId = RandomNumbers.nextInt(random, 0, 1000);
         int numberOfNavigableRegions = RandomNumbers.nextInt(random, 2, 10);
         for (int i = 0; i < numberOfNavigableRegions; i++)
            navigableRegions.add(VisibilityGraphRandomTools.getRandomNavigableRegion(random));

         VisibilityGraphStatistics statisticsToConvert = new VisibilityGraphStatistics();
         statisticsToConvert.setGoalVisibilityMapInWorld(goalMap.getMapId(), goalMap.getVisibilityMapInWorld());
         statisticsToConvert.setStartVisibilityMapInWorld(startMap.getMapId(), startMap.getVisibilityMapInWorld());
         statisticsToConvert.setInterRegionsVisibilityMapInWorld(interRegionsMap.getMapId(), interRegionsMap.getVisibilityMapInWorld());
         statisticsToConvert.setNavigableRegions(navigableRegions);

         BodyPathPlanStatisticsMessage message = VisibilityGraphMessagesConverter.convertToBodyPathPlanStatisticsMessage(planId, statisticsToConvert);
         VisibilityGraphStatistics convertedStatistics = VisibilityGraphMessagesConverter.convertToVisibilityGraphStatistics(message);
         VisibilityGraphTestTools.assertVisibilityGraphStatisticsEqual(statisticsToConvert, convertedStatistics, epsilon);
      }
   }
}
