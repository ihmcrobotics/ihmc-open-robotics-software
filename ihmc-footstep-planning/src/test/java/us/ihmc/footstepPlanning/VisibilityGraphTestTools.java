package us.ihmc.footstepPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.robotics.geometry.PlanarRegionTestTools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class VisibilityGraphTestTools
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static void assertVisibilityGraphStatisticsEqual(VisibilityGraphStatistics expected, VisibilityGraphStatistics actual, double epsilon)
   {
      assertEquals(expected.getGoalMapId(), actual.getGoalMapId());
      assertEquals(expected.getStartMapId(), actual.getStartMapId());
      assertEquals(expected.getInterRegionsMapId(), actual.getInterRegionsMapId());

      assertVisibilityMapsEqual(expected.getGoalVisibilityMap(), actual.getGoalVisibilityMap(), epsilon);
      assertVisibilityMapsEqual(expected.getStartVisibilityMap(), actual.getStartVisibilityMap(), epsilon);
      assertVisibilityMapsEqual(expected.getInterRegionsVisibilityMap(), actual.getInterRegionsVisibilityMap(), epsilon);

      assertEquals(expected.getNumberOfNavigableRegions(), actual.getNumberOfNavigableRegions());

      for (int i = 0; i < expected.getNumberOfNavigableRegions(); i++)
         assertNavigableRegionsEqual(expected.getNavigableRegion(i), actual.getNavigableRegion(i), epsilon);
   }

   public static void assertVisibilityMapHoldersEqual(VisibilityMapHolder holderExpected, VisibilityMapHolder holderActual, double epsilon)
   {
      assertEquals(holderExpected.getMapId(), holderActual.getMapId());
      assertVisibilityMapsEqual(holderExpected.getVisibilityMapInWorld(), holderActual.getVisibilityMapInWorld(), epsilon);
      assertVisibilityMapsEqual(holderExpected.getVisibilityMapInLocal(), holderActual.getVisibilityMapInLocal(), epsilon);

      for (Connection connection : holderExpected.getVisibilityMapInWorld().getConnections())
      {
         assertEquals(holderExpected.getConnectionWeight(connection), holderActual.getConnectionWeight(connection), epsilon);
      }

      for (Connection connection : holderExpected.getVisibilityMapInLocal().getConnections())
      {
         assertEquals(holderExpected.getConnectionWeight(connection), holderActual.getConnectionWeight(connection), epsilon);
      }
   }

   public static void assertSingleSourceVisibilityMapHoldersEqual(VisibilityMapHolder holderExpected, VisibilityMapHolder holderActual, double epsilon)
   {
      assertEquals(holderExpected.getMapId(), holderActual.getMapId());
      VisibilityGraphTestTools.assertVisibilityMapsEqual(holderExpected.getVisibilityMapInWorld(), holderActual.getVisibilityMapInWorld(), epsilon);

      for (Connection connection : holderExpected.getVisibilityMapInWorld().getConnections())
      {
         assertEquals(holderExpected.getConnectionWeight(connection), holderActual.getConnectionWeight(connection), epsilon);
      }
   }



   public static void assertVisibilityMapsEqual(VisibilityMap mapExpected, VisibilityMap mapActual, double epsilon)
   {
      assertEquals(mapExpected.getConnections().size(), mapActual.getConnections().size());
      assertEquals(mapExpected.getVertices().size(), mapActual.getVertices().size());
      Connection[] expectedConnections = mapExpected.getConnections().toArray(new Connection[0]);
      Connection[] actualConnections = mapActual.getConnections().toArray(new Connection[0]);
      ConnectionPoint3D[] expectedVertices = mapExpected.getVertices().toArray(new ConnectionPoint3D[0]);
      ConnectionPoint3D[] actualVertices = mapActual.getVertices().toArray(new ConnectionPoint3D[0]);
      for (int i = 0; i < mapExpected.getConnections().size(); i++)
         assertConnectionsEqual(expectedConnections[i], actualConnections[i], epsilon);
      for (int i = 0; i < mapExpected.getVertices().size(); i++)
         assertConnectionPointsEqual(expectedVertices[i], actualVertices[i], epsilon);

   }

   public static void assertConnectionsEqual(Connection connectionExpected, Connection connectionActual, double epsilon)
   {
      assertConnectionPointsEqual(connectionExpected.getSourcePoint(), connectionActual.getSourcePoint(), epsilon);
      assertConnectionPointsEqual(connectionExpected.getTargetPoint(), connectionActual.getTargetPoint(), epsilon);
   }

   public static void assertConnectionPointsEqual(ConnectionPoint3D pointExpected, ConnectionPoint3D pointActual, double epsilon)
   {
      assertEquals(pointExpected.getRegionId(), pointActual.getRegionId());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(pointExpected, pointActual, epsilon);
   }

   public static void assertClustersEqual(Cluster clusterExpected, Cluster clusterActual, double epsilon)
   {
      assertEquals(clusterExpected.getExtrusionSide(), clusterActual.getExtrusionSide());
      assertEquals(clusterExpected.getType(), clusterActual.getType());

      RigidBodyTransform transformExpected = clusterExpected.getTransformToWorld();
      ReferenceFrame localFrame = new ReferenceFrame("localFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformExpected);
         }
      };
      localFrame.update();
      EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(transformExpected, clusterActual.getTransformToWorld(), epsilon);

      int numberOfRawPoints = clusterExpected.getRawPointsInLocal2D().size();
      assertEquals(numberOfRawPoints, clusterActual.getNumberOfRawPoints());
      assertEquals(numberOfRawPoints, clusterExpected.getNumberOfRawPoints());
      assertEquals(numberOfRawPoints, clusterExpected.getRawPointsInLocal2D().size());
      assertEquals(numberOfRawPoints, clusterExpected.getRawPointsInWorld().size());
      assertEquals(numberOfRawPoints, clusterExpected.getRawPointsInLocal3D().size());
      assertEquals(numberOfRawPoints, clusterActual.getRawPointsInLocal2D().size());
      assertEquals(numberOfRawPoints, clusterActual.getRawPointsInWorld().size());
      assertEquals(numberOfRawPoints, clusterActual.getRawPointsInLocal3D().size());

      for (int i = 0; i < numberOfRawPoints; i++)
      {
         EuclidCoreTestTools
               .assertPoint3DGeometricallyEquals("Point " + i + " failed.", clusterExpected.getRawPointInLocal(i), clusterActual.getRawPointInLocal(i),
                                                 epsilon);
         EuclidCoreTestTools
               .assertPoint3DGeometricallyEquals("Point " + i + " failed.", clusterExpected.getRawPointInWorld(i), clusterActual.getRawPointInWorld(i),
                                                 epsilon);

         FramePoint3D point = new FramePoint3D(localFrame, clusterExpected.getRawPointInLocal(i));
         point.changeFrame(worldFrame);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Point " + i + " failed.", point, clusterExpected.getRawPointInWorld(i), epsilon);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Point " + i + " failed.", point, clusterActual.getRawPointInWorld(i), epsilon);
      }

      int numberOfNavigableExtrusions = clusterExpected.getNumberOfNavigableExtrusions();
      assertEquals(numberOfNavigableExtrusions, clusterActual.getNumberOfNavigableExtrusions());
      assertEquals(numberOfNavigableExtrusions, clusterExpected.getNavigableExtrusionsInLocal().size());
      assertEquals(numberOfNavigableExtrusions, clusterExpected.getNavigableExtrusionsInWorld().size());
      assertEquals(numberOfNavigableExtrusions, clusterActual.getNavigableExtrusionsInLocal().size());
      assertEquals(numberOfNavigableExtrusions, clusterActual.getNavigableExtrusionsInWorld().size());

      for (int i = 0; i < numberOfNavigableExtrusions; i++)
      {
         EuclidCoreTestTools
               .assertPoint2DGeometricallyEquals(clusterExpected.getNavigableExtrusionInLocal(i), clusterActual.getNavigableExtrusionInLocal(i), epsilon);
         EuclidCoreTestTools
               .assertPoint3DGeometricallyEquals(clusterExpected.getNavigableExtrusionInWorld(i), clusterActual.getNavigableExtrusionInWorld(i), epsilon);

         FramePoint3D framePoint = new FramePoint3D(localFrame, clusterExpected.getNavigableExtrusionInLocal(i));
         framePoint.changeFrame(worldFrame);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(framePoint, clusterExpected.getNavigableExtrusionInWorld(i), epsilon);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(framePoint, clusterActual.getNavigableExtrusionInWorld(i), epsilon);
      }

      int numberOfNonNavigableExtrusions = clusterExpected.getNumberOfNonNavigableExtrusions();
      assertEquals(numberOfNonNavigableExtrusions, clusterActual.getNumberOfNonNavigableExtrusions());
      assertEquals(numberOfNonNavigableExtrusions, clusterExpected.getNonNavigableExtrusionsInWorld().size());
      assertEquals(numberOfNonNavigableExtrusions, clusterExpected.getNonNavigableExtrusionsInLocal().size());
      assertEquals(numberOfNonNavigableExtrusions, clusterActual.getNonNavigableExtrusionsInWorld().size());
      assertEquals(numberOfNonNavigableExtrusions, clusterActual.getNonNavigableExtrusionsInLocal().size());

      for (int i = 0; i < numberOfNonNavigableExtrusions; i++)
      {
         EuclidCoreTestTools
               .assertPoint2DGeometricallyEquals(clusterExpected.getNonNavigableExtrusionInLocal(i), clusterActual.getNonNavigableExtrusionInLocal(i), epsilon);
         EuclidCoreTestTools
               .assertPoint3DGeometricallyEquals(clusterExpected.getNonNavigableExtrusionInWorld(i), clusterActual.getNonNavigableExtrusionInWorld(i), epsilon);

         FramePoint3D framePoint = new FramePoint3D(localFrame, clusterExpected.getNonNavigableExtrusionInLocal(i));
         framePoint.changeFrame(worldFrame);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(framePoint, clusterExpected.getNonNavigableExtrusionInWorld(i), epsilon);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(framePoint, clusterActual.getNonNavigableExtrusionInWorld(i), epsilon);

      }
   }

   public static void assertNavigableRegionsEqual(NavigableRegion expected, NavigableRegion actual, double epsilon)
   {
      assertEquals(expected.getMapId(), actual.getMapId());
      assertClustersEqual(expected.getHomeRegionCluster(), actual.getHomeRegionCluster(), epsilon);
      PlanarRegionTestTools.assertPlanarRegionsEqual(expected.getHomeRegion(), actual.getHomeRegion(), epsilon);
      assertEquals(expected.getObstacleClusters().size(), actual.getObstacleClusters().size());
      assertEquals(expected.getAllClusters().size(), actual.getAllClusters().size());

      for (int i = 0; i < expected.getObstacleClusters().size(); i++)
         assertClustersEqual(expected.getObstacleClusters().get(i), actual.getObstacleClusters().get(i), epsilon);
      for (int i = 0; i < expected.getAllClusters().size(); i++)
         assertClustersEqual(expected.getAllClusters().get(i), actual.getAllClusters().get(i), epsilon);

      assertVisibilityMapsEqual(expected.getVisibilityMapInWorld(), actual.getVisibilityMapInWorld(), epsilon);
      assertVisibilityMapsEqual(expected.getVisibilityMapInLocal(), actual.getVisibilityMapInLocal(), epsilon);
   }

}
