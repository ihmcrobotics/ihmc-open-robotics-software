package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public class NewPlannerGoalActionPolicy implements FootstepPlannerHeuristicActionPolicy
{
   private final List<PlannerGoalRecommendationListener> plannerGoalRecommendationListener = new ArrayList<>();
   private final SimplePlanarRegionFootstepNodeSnapper snapper;

   public NewPlannerGoalActionPolicy(SimplePlanarRegionFootstepNodeSnapper snapper)
   {
      this.snapper = snapper;
   }

   public void addPlannerGoalRecommendationListener(PlannerGoalRecommendationListener plannerGoalRecommendationListener)
   {
      if (plannerGoalRecommendationListener != null)
         this.plannerGoalRecommendationListener.add(plannerGoalRecommendationListener);
   }

   @Override
   public void performActionFromNewNode(FootstepNode newNode, FootstepNode previousNode)
   {
      FootstepNodeSnapData newNodeSnapData = snapper.snapFootstepNode(newNode);
      FootstepNodeSnapData previousSnapData = snapper.snapFootstepNode(previousNode);

      RigidBodyTransform newNodeTransform = new RigidBodyTransform();
      RigidBodyTransform previousNodeTransform = new RigidBodyTransform();

      FootstepNodeTools.getSnappedNodeTransform(newNode, newNodeSnapData.getSnapTransform(), newNodeTransform);
      FootstepNodeTools.getSnappedNodeTransform(previousNode, previousSnapData.getSnapTransform(), previousNodeTransform);

      SideDependentList<SimpleFootstep> doubleFootstepGoal = new SideDependentList<>();

      ConvexPolygon2D leftFoothold;
      ConvexPolygon2D rightFoothold;
      FramePose3D newLeftPose = new FramePose3D();
      FramePose3D newRightPose = new FramePose3D();

      if (newNode.getRobotSide().equals(RobotSide.LEFT))
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

      for (PlannerGoalRecommendationListener goalRecommendationListener : plannerGoalRecommendationListener)
         goalRecommendationListener.notifyWithPlannerGoalRecommendation(newPlannerGoal);
   }
}
