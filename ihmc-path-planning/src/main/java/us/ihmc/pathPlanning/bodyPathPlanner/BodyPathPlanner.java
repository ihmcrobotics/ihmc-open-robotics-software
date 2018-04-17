package us.ihmc.pathPlanning.bodyPathPlanner;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface BodyPathPlanner
{
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList);

   /**
    * Computes a new body path plan given a start and goal point.
    * Precondition: {@link #setPlanarRegionsList(PlanarRegionsList)} has been called
    */
   public void compute(Point2D startPoint, Point2D goalPoint);

   /**
    * Computes and packs a pose along the curve specified by
    * alpha, which goes from 0.0 (start) to 1.0 (goal).
    * This pose's x-axis aligned with the body path
    */
   public void getPointAlongPath(double alpha, Pose2D poseToPack);

   /**
    * Computes the pose along the curve closest to the given point
    * @return alpha corresponding to the packed point
    */
   public double getClosestPoint(Point2D point, Pose2D poseToPack);

   /**
    * Returns arc length of the body path from alpha to the goal point
    */
   public double computePathLength(double alpha);
}