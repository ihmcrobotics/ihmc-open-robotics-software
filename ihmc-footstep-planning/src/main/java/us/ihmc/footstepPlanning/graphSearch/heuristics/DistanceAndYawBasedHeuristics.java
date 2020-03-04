package us.ihmc.footstepPlanning.graphSearch.heuristics;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class DistanceAndYawBasedHeuristics extends CostToGoHeuristics
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder;

   private final Point2D midFootPoint = new Point2D();
   private final Pose3D projectionPose = new Pose3D();

   public DistanceAndYawBasedHeuristics(FootstepNodeSnapperReadOnly snapper, DoubleProvider weight, FootstepPlannerParametersReadOnly parameters, WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder)
   {
      super(weight::getValue, parameters::getIdealFootstepWidth, snapper);
      this.parameters = parameters;
      this.bodyPathPlanHolder = bodyPathPlanHolder;
   }

   @Override
   protected double computeHeuristics(FramePose3DReadOnly midFootPose3D)
   {
      double xyDistanceToGoal = EuclidCoreTools.norm(midFootPose3D.getX() - goalPose.getX(), midFootPose3D.getY() - goalPose.getY());

      double initialTurnDistance = 0.0;
      double walkDistance = 0.0;
      double finalTurnDistance;

      if(xyDistanceToGoal < parameters.getFinalTurnProximity())
      {
         finalTurnDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(midFootPose3D.getYaw(), goalPose.getYaw())) * 0.5 * Math.PI * parameters.getIdealFootstepWidth();
      }
      else
      {
         midFootPoint.set(midFootPose3D.getPosition());
         double alphaMidFoot = bodyPathPlanHolder.getClosestPoint(midFootPoint, projectionPose);
         int segmentIndex = bodyPathPlanHolder.getSegmentIndexFromAlpha(alphaMidFoot);
         double pathHeading = bodyPathPlanHolder.getSegmentYaw(segmentIndex);

         initialTurnDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(midFootPose3D.getYaw(), pathHeading)) * 0.5 * Math.PI * parameters.getIdealFootstepWidth();
         walkDistance = xyDistanceToGoal;
         finalTurnDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(pathHeading, goalPose.getYaw())) * 0.5 * Math.PI * parameters.getIdealFootstepWidth();
     }

      return initialTurnDistance + walkDistance + finalTurnDistance;
   }
}
