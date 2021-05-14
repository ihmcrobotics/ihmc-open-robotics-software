package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FootstepPlannerHeuristicCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FootstepSnapperReadOnly snapper;

   private final FootstepPlannerParametersReadOnly parameters;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder;
   private double desiredHeading;

   private final Pose3D projectionPose = new Pose3D();
   private final Pose3D goalPose = new Pose3D();

   public FootstepPlannerHeuristicCalculator(FootstepSnapperReadOnly snapper,
                                             FootstepPlannerParametersReadOnly parameters,
                                             WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder,
                                             YoRegistry parentRegistry)
   {
      this.snapper = snapper;

      this.parameters = parameters;
      this.bodyPathPlanHolder = bodyPathPlanHolder;
      parentRegistry.addChild(registry);
   }

   public void initialize(FramePose3DReadOnly goalPose, double desiredHeading)
   {
      this.goalPose.set(goalPose);
      this.desiredHeading = desiredHeading;
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
         double pathHeading = EuclidCoreTools.trimAngleMinusPiToPi(bodyPathPlanHolder.getSegmentYaw(segmentIndex) + desiredHeading);

         initialTurnDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(midFootPose.getYaw(), pathHeading)) * 0.5 * Math.PI * parameters.getIdealFootstepWidth();
         walkDistance = xyDistanceToGoal;
         finalTurnDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(pathHeading, goalPose.getYaw())) * 0.5 * Math.PI * parameters.getIdealFootstepWidth();
     }

      return parameters.getAStarHeuristicsWeight().getValue() * (initialTurnDistance + walkDistance + finalTurnDistance);
   }
}
