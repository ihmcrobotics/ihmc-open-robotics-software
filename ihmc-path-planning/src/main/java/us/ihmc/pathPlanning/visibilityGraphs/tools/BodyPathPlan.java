package us.ihmc.pathPlanning.visibilityGraphs.tools;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class BodyPathPlan
{
   private final ArrayList<Point2DReadOnly> bodyPathWaypoints = new ArrayList<>();
   private final Pose2D startPose = new Pose2D();
   private final Pose2D goalPose = new Pose2D();

   public BodyPathPlan()
   {
   }

   public int getNumberOfWaypoints()
   {
      return bodyPathWaypoints.size();
   }

   public Point2DReadOnly getWaypoint(int waypointIndex)
   {
      return bodyPathWaypoints.get(waypointIndex);
   }

   public Point2D addWaypoint(Point2DReadOnly waypoint)
   {
      Point2D waypointCopy = new Point2D(waypoint);
      bodyPathWaypoints.add(waypointCopy);

      return waypointCopy;
   }

   public void addWaypoints(List<? extends Point2DReadOnly> waypoints)
   {
      for (int i = 0; i < waypoints.size(); i++)
         addWaypoint(waypoints.get(i));
   }

   public void clear()
   {
      bodyPathWaypoints.clear();
   }

   public void setStartPose(Pose2DReadOnly startPose)
   {
      this.startPose.set(startPose);
   }

   public void setStartPose(Point2DReadOnly startPosition, double startYaw)
   {
      this.startPose.set(startPosition, startYaw);
   }

   public void setGoalPose(Pose2DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }

   public void setGoalPose(Point2DReadOnly goalPosition, double goalYaw)
   {
      this.goalPose.set(goalPosition, goalYaw);
   }

   public Pose2DReadOnly getStartPose()
   {
      return startPose;
   }

   public Pose2DReadOnly getGoalPose()
   {
      return goalPose;
   }
}
