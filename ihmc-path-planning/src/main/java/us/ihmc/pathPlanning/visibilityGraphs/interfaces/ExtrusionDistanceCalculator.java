package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;

public interface ExtrusionDistanceCalculator
{
   /**
    * @param clusterInExtrusion the cluster being extruded. Do not modify.
    * @param pointToExtrude the coordinates of the point being extruded. Do not modify.
    * @param obstacleHeight the height of the obstacle from which the point to extrude is
    *           created.
    * @return positive value representing the ditance between the raw points of a cluster and the
    *         extrusion.
    */
   double computeExtrusionDistance(Cluster clusterInExtrusion, Point2DReadOnly pointToExtrude, double obstacleHeight);
}