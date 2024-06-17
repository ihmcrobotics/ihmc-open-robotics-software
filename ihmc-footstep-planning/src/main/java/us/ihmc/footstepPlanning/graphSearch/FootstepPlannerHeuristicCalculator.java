package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FootstepPlannerHeuristicCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DefaultFootstepPlannerParametersReadOnly parameters;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder;

   private final Pose3D projectionPose = new Pose3D();
   private final Pose3D goalPose = new Pose3D();

   public FootstepPlannerHeuristicCalculator(DefaultFootstepPlannerParametersReadOnly parameters,
                                             WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder,
                                             YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.bodyPathPlanHolder = bodyPathPlanHolder;
      parentRegistry.addChild(registry);
   }

   public void initialize(FramePose3DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }

   public double compute(FootstepGraphNode node)
   {
      Pose2D midFootPose = node.getOrComputeMidFootPose();

      double xyDistanceToGoal = EuclidCoreTools.norm(midFootPose.getX() - goalPose.getX(), midFootPose.getY() - goalPose.getY());

      double initialTurnDistance = 0.0;
      double walkDistance = 0.0;
      double finalTurnDistance;

      if(xyDistanceToGoal < parameters.getFinalTurnProximity())
      {
         finalTurnDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(midFootPose.getYaw(), goalPose.getYaw())) * 0.5 * Math.PI * parameters.getIdealFootstepWidth();
      }
      else
      {
         double alphaMidFoot = bodyPathPlanHolder.getClosestPoint(midFootPose.getPosition(), projectionPose);
         int segmentIndex = bodyPathPlanHolder.getSegmentIndexFromAlpha(alphaMidFoot);

         double desiredRobotPostureHeading = bodyPathPlanHolder.getBodyPathPlan().getWaypoint(segmentIndex).getOrientation().getYaw();
         double finalRobotPostureHeading = bodyPathPlanHolder.getBodyPathPlan().getWaypoint(bodyPathPlanHolder.getBodyPathPlan().getNumberOfWaypoints() - 1).getYaw();

         initialTurnDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(midFootPose.getYaw(), desiredRobotPostureHeading)) * 0.5 * Math.PI * parameters.getIdealFootstepWidth();
         walkDistance = xyDistanceToGoal;
         finalTurnDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(finalRobotPostureHeading, goalPose.getYaw())) * 0.5 * Math.PI * parameters.getIdealFootstepWidth();
     }

      return parameters.getAStarHeuristicsWeight() * (initialTurnDistance + walkDistance + finalTurnDistance);
   }
}
