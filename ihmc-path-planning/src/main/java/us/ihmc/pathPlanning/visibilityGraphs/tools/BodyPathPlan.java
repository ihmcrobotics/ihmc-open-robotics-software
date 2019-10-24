package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class BodyPathPlan
{
   private final ArrayList<Pose3DReadOnly> bodyPathWaypoints = new ArrayList<>();
   private final Pose2D startPose = new Pose2D();
   private final Pose2D goalPose = new Pose2D();

   public BodyPathPlan()
   {
   }

   public int getNumberOfWaypoints()
   {
      return bodyPathWaypoints.size();
   }

   public Pose3DReadOnly getWaypoint(int waypointIndex)
   {
      return bodyPathWaypoints.get(waypointIndex);
   }

   public Pose3DReadOnly addWaypoint(Pose3DReadOnly waypoint)
   {
      Pose3D waypointCopy = new Pose3D(waypoint);
      bodyPathWaypoints.add(waypointCopy);

      return waypointCopy;
   }

   public void addWaypoints(List<? extends Pose3DReadOnly> waypoints)
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
      setStartPose(startPosition.getX(), startPosition.getY(), startYaw);
   }

   public void setStartPose(double startXPosition, double startYPosition, double startYaw)
   {
      this.startPose.set(startXPosition, startYPosition, startYaw);
   }

   public void setGoalPose(Pose2DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }

   public void setGoalPose(Point2DReadOnly goalPosition, double goalYaw)
   {
      setGoalPose(goalPosition.getX(), goalPosition.getY(), goalYaw);
   }

   public void setGoalPose(double goalXPosition, double goalYPosition, double goalYaw)
   {
      this.goalPose.set(goalXPosition, goalYPosition, goalYaw);
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
