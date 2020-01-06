package us.ihmc.valkyrie.planner;

import controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

class BodyPathHelper
{
   private final ValkyrieAStarFootstepPlannerParameters parameters;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();
   private final List<WaypointData> waypointHolder = new ArrayList<>();
   private final FramePose3D projectionPoint = new FramePose3D();

   private double pathLength;
   private boolean waypointMode;

   BodyPathHelper(ValkyrieAStarFootstepPlannerParameters parameters)
   {
      this.parameters = parameters;
   }

   void initialize(ValkyrieFootstepPlanningRequestPacket requestPacket)
   {
      waypointMode = !requestPacket.getWaypoints().isEmpty();
      waypointHolder.clear();

      FramePose3D startMidFootPose = new FramePose3D();
      startMidFootPose.interpolate(requestPacket.getStartLeftFootPose(), requestPacket.getStartRightFootPose(), 0.5);
      FramePose3D goalMidFootPose = new FramePose3D();
      goalMidFootPose.interpolate(requestPacket.getGoalLeftFootPose(), requestPacket.getGoalRightFootPose(), 0.5);

      if (waypointMode)
      {
         bodyPathPlanHolder.getPlan().clear();
         List<Pose3D> bodyPath = new ArrayList<>();
         bodyPath.add(new Pose3D(startMidFootPose));
         bodyPath.addAll(requestPacket.getWaypoints());
         bodyPath.add(new Pose3D(goalMidFootPose));
         bodyPathPlanHolder.setPoseWaypoints(bodyPath);

         pathLength = bodyPathPlanHolder.computePathLength(0.0);
      }

      for (int i = 0; i < requestPacket.getWaypoints().size() + 1; i++)
      {
         Pose3DBasics startPose = (i == 0) ? startMidFootPose : requestPacket.getWaypoints().get(i - 1);
         if(i < requestPacket.getWaypoints().size())
         {
            waypointHolder.add(new WaypointData(i, startPose, requestPacket.getWaypoints().get(i)));
         }
         else
         {
            waypointHolder.add(new WaypointData(i, startPose, requestPacket.getGoalLeftFootPose(), requestPacket.getGoalRightFootPose()));
         }
      }
   }

   WaypointData getWaypointFromNode(FootstepNode node, double lookAheadDistance)
   {
      if (waypointMode)
      {
         Point2D midFootPoint = node.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
         double alpha = bodyPathPlanHolder.getClosestPoint(midFootPoint, projectionPoint);
         alpha = Math.min(1.0, alpha + lookAheadDistance / pathLength);

         int index = bodyPathPlanHolder.getSegmentIndexFromAlpha(alpha);
         return waypointHolder.get(index);
      }
      else
      {
         return waypointHolder.get(0);
      }
   }

   boolean hasWaypoints()
   {
      return waypointMode;
   }

   int getNumberOfPathSegments()
   {
      return waypointHolder.size();
   }

   WaypointData getWaypoint(int index)
   {
      return waypointHolder.get(index);
   }

   WaypointDefinedBodyPathPlanHolder getBodyPathPlanHolder()
   {
      return bodyPathPlanHolder;
   }

   class WaypointData
   {
      final int index;
      final double pathHeading;
      final Line2D pathLine = new Line2D();
      final Pose2D nominalGoalPose = new Pose2D();
      final SideDependentList<FootstepNode> goalNodes = new SideDependentList<>();

      public WaypointData(int index, Pose3DBasics startPose, Pose3DBasics nominalGoalPose)
      {
         this.index = index;
         this.nominalGoalPose.set(nominalGoalPose);
         this.pathLine.set(startPose.getPosition().getX(), startPose.getPosition().getY(), nominalGoalPose.getX() - startPose.getX(), nominalGoalPose.getY() - startPose.getY());
         this.pathHeading = Math.atan2(pathLine.getDirectionY(), pathLine.getDirectionX());
         this.goalNodes.set(side ->
                            {
                               Pose2D goalStepPose = new Pose2D(nominalGoalPose);
                               goalStepPose.appendTranslation(0.0, 0.5 * side.negateIfRightSide(parameters.getIdealFootstepWidth()));
                               return new FootstepNode(goalStepPose.getX(), goalStepPose.getY(), goalStepPose.getYaw(), side);
                            });
      }

      public WaypointData(int index, Pose3DBasics startPose, Pose3DBasics leftGoalStepPose, Pose3D rightGoalStepPose)
      {
         this.index = index;
         this.nominalGoalPose.interpolate(new Pose2D(leftGoalStepPose), new Pose2D(rightGoalStepPose), 0.5);
         this.pathLine.set(startPose.getPosition().getX(), startPose.getPosition().getY(), nominalGoalPose.getX() - startPose.getX(), nominalGoalPose.getY() - startPose.getY());
         this.pathHeading = Math.atan2(pathLine.getDirectionY(), pathLine.getDirectionX());
         this.goalNodes.set(side ->
                            {
                               Pose3DBasics goalStepPose = side == RobotSide.LEFT ? leftGoalStepPose : rightGoalStepPose;
                               return new FootstepNode(goalStepPose.getX(), goalStepPose.getY(), goalStepPose.getYaw(), side);
                            });
      }

      public int getIndex()
      {
         return index;
      }

      double getPathHeading()
      {
         return pathHeading;
      }

      Line2D getPathLine()
      {
         return pathLine;
      }

      Pose2DBasics getNominalGoalPose()
      {
         return nominalGoalPose;
      }

      SideDependentList<FootstepNode> getGoalNodes()
      {
         return goalNodes;
      }
   }
}