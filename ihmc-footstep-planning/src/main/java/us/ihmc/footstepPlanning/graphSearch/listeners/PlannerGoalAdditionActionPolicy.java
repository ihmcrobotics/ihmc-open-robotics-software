package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public class PlannerGoalAdditionActionPolicy implements PlannerHeuristicNodeActionPolicy
{
   private final List<PlannerGoalRecommendationListener> plannerGoalRecommendationListener = new ArrayList<>();
   private final FootstepNodeSnapper snapper;

   public PlannerGoalAdditionActionPolicy(FootstepNodeSnapper snapper)
   {
      this.snapper = snapper;
   }

   @Override
   public void addActionListener(PlannerHeuristicActionListener listener)
   {
      if (listener instanceof PlannerGoalRecommendationListener)
         this.plannerGoalRecommendationListener.add((PlannerGoalRecommendationListener) listener);
   }

   @Override
   public void performActionForNewValidNode(FootstepNode newValidNode, FootstepNode newValidParentNode)
   {
      FootstepNodeSnapData newNodeSnapData = snapper.snapFootstepNode(newValidNode);
      FootstepNodeSnapData previousSnapData = snapper.snapFootstepNode(newValidParentNode);

      RigidBodyTransform newNodeTransform = new RigidBodyTransform();
      RigidBodyTransform previousNodeTransform = new RigidBodyTransform();

      if(newNodeSnapData.getSnapTransform().containsNaN())
      {
         // assume flat ground in case of bad snap
         FootstepNodeTools.getNodeTransform(newValidNode, newNodeTransform);
      }
      else
      {
         newNodeTransform.set(newNodeSnapData.getOrComputeSnappedNodeTransform(newValidNode));
      }

      if(previousSnapData.getSnapTransform().containsNaN())
      {
         // assume flat ground in case of bad snap
         FootstepNodeTools.getNodeTransform(newValidParentNode, previousNodeTransform);
      }
      else
      {
         previousNodeTransform.set(previousSnapData.getOrComputeSnappedNodeTransform(newValidParentNode));
      }

      SideDependentList<SimpleFootstep> doubleFootstepGoal = new SideDependentList<>();

      ConvexPolygon2D leftFoothold;
      ConvexPolygon2D rightFoothold;
      FramePose3D newLeftPose = new FramePose3D();
      FramePose3D newRightPose = new FramePose3D();

      if (newValidNode.getRobotSide().equals(RobotSide.LEFT))
      {
         newLeftPose.appendTransform(newNodeTransform);
         newRightPose.appendTransform(previousNodeTransform);

         leftFoothold = newNodeSnapData.getCroppedFoothold();
         rightFoothold = previousSnapData.getCroppedFoothold();

      }
      else
      {
         newLeftPose.appendTransform(previousNodeTransform);
         newRightPose.appendTransform(newNodeTransform);

         leftFoothold = previousSnapData.getCroppedFoothold();
         rightFoothold = newNodeSnapData.getCroppedFoothold();
      }

      SimpleFootstep leftStep = new SimpleFootstep(RobotSide.LEFT, newLeftPose);
      SimpleFootstep rightStep = new SimpleFootstep(RobotSide.RIGHT, newRightPose);

      leftStep.setFoothold(leftFoothold);
      rightStep.setFoothold(rightFoothold);

      doubleFootstepGoal.put(RobotSide.LEFT, leftStep);
      doubleFootstepGoal.put(RobotSide.RIGHT, rightStep);

      FootstepPlannerGoal newPlannerGoal = new FootstepPlannerGoal();
      newPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.DOUBLE_FOOTSTEP);
      newPlannerGoal.setDoubleFootstepGoal(doubleFootstepGoal);

      for (PlannerGoalRecommendationListener listener : plannerGoalRecommendationListener)
         listener.notifyWithPlannerGoalRecommendation(newPlannerGoal, newValidNode.getRobotSide().getOppositeSide());
   }
}
