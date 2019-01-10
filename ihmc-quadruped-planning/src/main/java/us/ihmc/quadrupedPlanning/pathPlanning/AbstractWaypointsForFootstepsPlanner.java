package us.ihmc.quadrupedPlanning.pathPlanning;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedPlanning.footstepPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.quadrupedPlanning.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

public abstract class AbstractWaypointsForFootstepsPlanner implements WaypointsForQuadrupedFootstepPlanner
{
   protected final static boolean debug = false;

   protected final FramePose3D bodyStartPose = new FramePose3D();
   protected final FramePose3D bodyGoalPose = new FramePose3D();

   protected final List<Point3D> waypoints = new ArrayList<>();

   protected final YoEnum<FootstepPlanningResult> yoResult;

   protected PlanarRegionsList planarRegionsList;

   public AbstractWaypointsForFootstepsPlanner(String prefix, YoVariableRegistry registry)
   {
      yoResult = new YoEnum<>(prefix + "PathPlanningResult", registry, FootstepPlanningResult.class);
   }

   @Override
   public void setInitialBodyPose(FramePose3DReadOnly initialPose)
   {
      bodyStartPose.setToZero(ReferenceFrame.getWorldFrame());
      bodyStartPose.setPosition(initialPose.getX(), initialPose.getY(), 0.0);
      bodyStartPose.setOrientationYawPitchRoll(initialPose.getYaw(), 0.0, 0.0);
   }

   public void setGoal(QuadrupedFootstepPlannerGoal goal)
   {
      FramePose3DReadOnly goalPose = goal.getGoalPose();
      bodyGoalPose.setIncludingFrame(goalPose);
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void computeBestEffortPlan(double horizonLength)
   {
      Vector2D goalDirection = new Vector2D(bodyGoalPose.getPosition());
      goalDirection.sub(bodyStartPose.getX(), bodyStartPose.getY());
      goalDirection.scale(horizonLength / goalDirection.length());
      Point3D waypoint = new Point3D(bodyStartPose.getPosition());
      waypoint.add(goalDirection.getX(), goalDirection.getY(), 0.0);
      waypoints.add(waypoint);
   }

   public List<Point3D> getWaypoints()
   {
      return waypoints;
   }

   @Override
   public FramePose3DReadOnly getInitialBodyPose()
   {
      return bodyStartPose;
   }

   @Override
   public FramePose3DReadOnly getGoalBodyPose()
   {
      return bodyGoalPose;
   }

}
