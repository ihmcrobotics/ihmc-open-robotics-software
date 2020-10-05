package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.footstepPlanning.FootstepPlanHeading;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapDataReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootstepPlannerHeuristicCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FootstepNodeSnapperReadOnly snapper;

   private final FootstepPlannerParametersReadOnly parameters;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder;
   private double desiredHeading;

   private final FramePose3D midFootPose = new FramePose3D();
   private final Point2D midFootPoint = new Point2D();
   private final Pose3D projectionPose = new Pose3D();
   private final FramePose3D goalPose = new FramePose3D();
   private final Point3DBasics midfootPoint = new Point3D();

   public FootstepPlannerHeuristicCalculator(FootstepNodeSnapperReadOnly snapper,
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

   public double compute(FootstepNode node)
   {
      midfootPoint.set(node.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()), 0.0);

      FootstepNodeSnapDataReadOnly snapData = snapper.snapFootstepNode(node);
      if (snapData != null && !snapData.getSnapTransform().containsNaN())
      {
         snapData.getSnapTransform().transform(midfootPoint);
      }

      midFootPose.getPosition().set(midfootPoint);
      midFootPose.getOrientation().setYawPitchRoll(node.getYaw(), 0.0, 0.0);

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
         midFootPoint.set(midFootPose.getPosition());
         double alphaMidFoot = bodyPathPlanHolder.getClosestPoint(midFootPoint, projectionPose);
         int segmentIndex = bodyPathPlanHolder.getSegmentIndexFromAlpha(alphaMidFoot);
         double pathHeading = EuclidCoreTools.trimAngleMinusPiToPi(bodyPathPlanHolder.getSegmentYaw(segmentIndex) + desiredHeading);

         initialTurnDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(midFootPose.getYaw(), pathHeading)) * 0.5 * Math.PI * parameters.getIdealFootstepWidth();
         walkDistance = xyDistanceToGoal;

         /** TODO remove when {@link FootstepPlannerParametersReadOnly#getStepOnlyWithRequestedSide()} is removed */
         if (node.getRobotSide() == parameters.getStepOnlyWithRequestedSide())
         {
            walkDistance += 0.5 * parameters.getIdealFootstepLength();
         }

         finalTurnDistance = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(pathHeading, goalPose.getYaw())) * 0.5 * Math.PI * parameters.getIdealFootstepWidth();
     }

      return parameters.getAStarHeuristicsWeight().getValue() * (initialTurnDistance + walkDistance + finalTurnDistance);
   }
}
