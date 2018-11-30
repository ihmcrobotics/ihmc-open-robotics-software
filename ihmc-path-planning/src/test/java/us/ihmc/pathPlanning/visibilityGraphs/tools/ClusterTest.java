package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;

public class ClusterTest
{

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testIsInsideNonNavigableZone()
   {
      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      Point2D pointD = new Point2D(0.0, 1.0);

      Cluster obstacleCluster = new Cluster();

      obstacleCluster.addNonNavigableExtrusionInLocal(new Point2D(0.1, -0.1));
      obstacleCluster.addNonNavigableExtrusionInLocal(new Point2D(1.1, -0.1));
      obstacleCluster.addNonNavigableExtrusionInLocal(new Point2D(1.1, 1.1));
      obstacleCluster.addNonNavigableExtrusionInLocal(new Point2D(0.1, 1.1));

      assertFalse(obstacleCluster.isInsideNonNavigableZone(pointA));
      assertTrue(obstacleCluster.isInsideNonNavigableZone(pointB));
      assertTrue(obstacleCluster.isInsideNonNavigableZone(pointC));
      assertFalse(obstacleCluster.isInsideNonNavigableZone(pointD));
   }

}
