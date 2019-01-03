package us.ihmc.pathPlanning.bodyPathPlanner;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface BodyPathPlanner
{
   /** Adds the waypoints used by the body path planner. **/
   void setWaypoints(List<? extends Point3DReadOnly> waypoints);

   /** This method is now completely unused. **/
   @Deprecated
   default void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
   }

   /**
    * Computes a new body path plan given a start and goal point.
    * Precondition: {@link #setPlanarRegionsList(PlanarRegionsList)} has been called.
    * This has been deprecated. Use {@link #compute()} along with {@link #setWaypoints(List)} instead
    */
   @Deprecated
   default void compute(Point2D startPoint, Point2D goalPoint)
   {
      List<Point3DReadOnly> waypoints = new ArrayList<>();
      waypoints.add(new Point3D(startPoint));
      waypoints.add(new Point3D(goalPoint));
      setWaypoints(waypoints);
      compute();
   }

   /**
    * Computes a new body path plan given a start and goal point.
    * Precondition: {@link #setPlanarRegionsList(PlanarRegionsList)} has been called
    */
   BodyPathPlan compute();

   /**
    * Gets the computed body path plan.
    */
   default BodyPathPlan getPlan()
   {
      return null;
   }

   /**
    * Computes and packs a pose along the curve specified by
    * alpha, which goes from 0.0 (start) to 1.0 (goal).
    * This pose's x-axis aligned with the body path
    */
   void getPointAlongPath(double alpha, Pose2D poseToPack);

   /**
    * Computes the pose along the curve closest to the given point
    * @return alpha corresponding to the packed point
    */
   double getClosestPoint(Point2D point, Pose2D poseToPack);

   /**
    * Returns arc length of the body path from alpha to the goal point
    */
   double computePathLength(double alpha);
}