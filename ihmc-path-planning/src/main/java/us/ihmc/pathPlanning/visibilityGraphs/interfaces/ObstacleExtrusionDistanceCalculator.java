package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public interface ObstacleExtrusionDistanceCalculator
{
   /**
    * @param pointToExtrude the coordinates of the point being extruded. Do not modify.
    * @param obstacleHeight the height of the obstacle from which the point to extrude is
    *           created.
    * @return positive value representing the ditance between the raw points of a cluster and the
    *         extrusion.
    */
   double computeExtrusionDistance(Point2DReadOnly pointToExtrude, double obstacleHeight);
}