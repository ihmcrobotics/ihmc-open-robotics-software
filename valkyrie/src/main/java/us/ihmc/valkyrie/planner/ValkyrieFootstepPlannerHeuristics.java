package us.ihmc.valkyrie.planner;

import controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class ValkyrieFootstepPlannerHeuristics
{
   private final DistanceAndYawBasedHeuristics distanceAndYawHeuristics;
   private final ValkyrieFootstepPlanningRequestPacket requestPacket = new ValkyrieFootstepPlanningRequestPacket();
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();
   private final ValkyrieAStarFootstepPlannerParameters parameters;

   private final FramePose3D startMidFootPose = new FramePose3D();
   private final FramePose3D goalMidFootPose = new FramePose3D();
   private boolean waypointMode = false;
   private double dAlpha = 0.0;

   public ValkyrieFootstepPlannerHeuristics(ValkyrieAStarFootstepPlannerParameters parameters, FootstepNodeSnapperReadOnly snapper)
   {
      distanceAndYawHeuristics = new DistanceAndYawBasedHeuristics(parameters.getTranslationWeight()::getZ,
                                                                   parameters.getTranslationWeight()::getZ,
                                                                   parameters::getMaximumStepReach,
                                                                   parameters::getMaximumStepYaw,
                                                                   parameters.getOrientationWeight()::getYaw,
                                                                   parameters::getCostPerStep,
                                                                   parameters::getFinalTurnProximity,
                                                                   () -> 0.25,
                                                                   parameters::getIdealFootstepWidth,
                                                                   parameters::getAstarHeuristicsWeight,
                                                                   snapper);
      this.parameters = parameters;
   }

   public void setRequestPacket(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      this.requestPacket.set(requestPacket);
      waypointMode = !requestPacket.getWaypoints().isEmpty();
      startMidFootPose.interpolate(requestPacket.getStartLeftFootPose(), requestPacket.getStartRightFootPose(), 0.5);
      goalMidFootPose.interpolate(requestPacket.getGoalLeftFootPose(), requestPacket.getGoalRightFootPose(), 0.5);

      if (waypointMode)
      {
         bodyPathPlanHolder.getPlan().clear();
         List<Pose3D> bodyPath = new ArrayList<>();
         bodyPath.add(new Pose3D(startMidFootPose));
         bodyPath.addAll(requestPacket.getWaypoints());
         bodyPath.add(new Pose3D(goalMidFootPose));
         bodyPathPlanHolder.setPoseWaypoints(bodyPath);
         dAlpha = 0.1 / bodyPathPlanHolder.computePathLength(0.0);
      }
      else
      {
         distanceAndYawHeuristics.setGoalPose(goalMidFootPose);
      }
   }

   private final FramePose3D projectionPoint = new FramePose3D();
   private final FramePose3D goalPose = new FramePose3D();

   public double compute(FootstepNode node)
   {
      if (waypointMode)
      {
         Point2D midFootPoint = node.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
         double alpha = bodyPathPlanHolder.getClosestPoint(midFootPoint, projectionPoint);
         alpha = Math.min(1.0, alpha + dAlpha);

         int index = bodyPathPlanHolder.getSegmentIndexFromAlpha(alpha);
         if (index >= requestPacket.getWaypoints().size())
         {
            goalPose.set(goalMidFootPose);
         }
         else
         {
            goalPose.set(requestPacket.getWaypoints().get(index));
         }

         distanceAndYawHeuristics.setGoalPose(goalPose);
         double intermediateHeuristics = distanceAndYawHeuristics.compute(node);
         double remainingDistance = bodyPathPlanHolder.computePathLength(bodyPathPlanHolder.getMaxAlphaFromSegmentIndex(index));
         double remainingHeuristics = remainingDistance * parameters.getAstarHeuristicsWeight() * parameters.getCostPerStep() / parameters.getMaximumStepReach();
         return intermediateHeuristics + remainingHeuristics;
      }
      else
      {
         return distanceAndYawHeuristics.compute(node);
      }
   }
}
