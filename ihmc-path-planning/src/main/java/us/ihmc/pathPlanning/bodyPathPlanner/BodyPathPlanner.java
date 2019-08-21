package us.ihmc.pathPlanning.bodyPathPlanner;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface BodyPathPlanner
{
   /** Adds the position waypoints used by the body path planner, and automatically computes the heading waypoints. **/
   default void setWaypoints(List<? extends Point3DReadOnly> positionWaypoints)
   {
      int numberOfSegments = positionWaypoints.size() - 1;
      List<MutableDouble> headingWaypoints = new ArrayList<>();
      double startingHeading = BodyPathPlannerTools.calculateHeading(positionWaypoints.get(0), positionWaypoints.get(1));
      headingWaypoints.add(new MutableDouble(startingHeading));

      for (int i = 0; i < numberOfSegments - 1; i++)
      {
         double headingBefore = BodyPathPlannerTools.calculateHeading(positionWaypoints.get(i), positionWaypoints.get(i + 1));
         double headingAfter = BodyPathPlannerTools.calculateHeading(positionWaypoints.get(i + 1), positionWaypoints.get(i + 2));
         double heading = AngleTools.interpolateAngle(headingBefore, headingAfter, 0.5);
         headingWaypoints.add(new MutableDouble(heading));
      }

      double endingHeading = BodyPathPlannerTools.calculateHeading(positionWaypoints.get(numberOfSegments - 1), positionWaypoints.get(numberOfSegments));
      headingWaypoints.add(new MutableDouble(endingHeading));

      setWaypoints(positionWaypoints, headingWaypoints);
   }

   /** Adds the position and heading waypoints used by the body path planner. **/
   void setWaypoints(List<? extends Point3DReadOnly> positionWaypoints, List<MutableDouble> headingWaypoints);

   /** This method is now completely unused. **/
   @Deprecated
   default void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
   }

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
   double getClosestPoint(Point2DReadOnly point, Pose2D poseToPack);

   /**
    * Returns arc length of the body path from alpha to the goal point
    */
   double computePathLength(double alpha);
}