package us.ihmc.footstepPlanning.graphSearch.pathPlanners;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

public abstract class AbstractWaypointsForFootstepsPlanner implements WaypointsForFootstepsPlanner
{
   protected static final boolean debug = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;

   protected final YoEnum<FootstepPlanningResult> yoResult;

   protected final FootstepPlannerParametersReadOnly parameters;
   protected final FramePose3D bodyStartPose = new FramePose3D();
   protected final FramePose3D bodyGoalPose = new FramePose3D();

   protected final List<Point3D> waypoints = new ArrayList<>();

   protected PlanarRegionsList planarRegionsList;

   public AbstractWaypointsForFootstepsPlanner(FootstepPlannerParametersReadOnly parameters, YoVariableRegistry registry)
   {
      this("", parameters, registry);
   }

   public AbstractWaypointsForFootstepsPlanner(String prefix, FootstepPlannerParametersReadOnly parameters, YoVariableRegistry registry)
   {
      this.parameters = parameters;

      yoResult = new YoEnum<>(prefix + "PathPlanningResult", registry, FootstepPlanningResult.class);
   }

   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      if (side == null)
      {
         if (debug)
            PrintTools.info("Start node needs a side, but trying to set it to null. Setting it to " + defaultStartNodeSide);

         side = defaultStartNodeSide;
      }

      double defaultStepWidth = parameters.getIdealFootstepWidth();
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);
      FramePoint2D bodyStartPoint = new FramePoint2D(stanceFrame);
      bodyStartPoint.setY(side.negateIfLeftSide(defaultStepWidth / 2.0));
      bodyStartPoint.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      bodyStartPose.setToZero(ReferenceFrame.getWorldFrame());
      bodyStartPose.setPosition(bodyStartPoint.getX(), bodyStartPoint.getY(), 0.0);
      bodyStartPose.setOrientationYawPitchRoll(stanceFootPose.getYaw(), 0.0, 0.0);
   }

   public void setGoal(FootstepPlannerGoal goal)
   {
      AStarFootstepPlanner.checkGoalType(goal);
      FramePose3D goalPose = goal.getGoalPoseBetweenFeet();
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
}
