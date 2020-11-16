package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanWithTextGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.ArrayList;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.BodyPathPlanForUI;

public class LookAndStepVisualizationGroup extends Group
{
   private boolean reviewingBodyPath = true;

   private FootstepPlanWithTextGraphic footstepPlanGraphic;
   private FootstepPlanWithTextGraphic commandedFootsteps;
   private FootstepPlanWithTextGraphic startAndGoalFootPoses;
   private PoseGraphic closestPointAlongPathGraphic;
   private PoseGraphic subGoalGraphic;
   private BodyPathPlanGraphic bodyPathPlanGraphic;
   private LivePlanarRegionsGraphic planarRegionsGraphic;
   private PoseGraphic goalGraphic;

   public LookAndStepVisualizationGroup(ROS2NodeInterface ros2Node, Messager behaviorMessager)
   {
      startAndGoalFootPoses = new FootstepPlanWithTextGraphic();
      startAndGoalFootPoses.setColor(RobotSide.LEFT, Color.BLUE);
      startAndGoalFootPoses.setColor(RobotSide.RIGHT, Color.BLUE);
      startAndGoalFootPoses.setTransparency(0.4);
      behaviorMessager.registerTopicListener(StartAndGoalFootPosesForUI, startAndGoalFootPoses::generateMeshesAsynchronously);
      footstepPlanGraphic = new FootstepPlanWithTextGraphic();
      footstepPlanGraphic.setTransparency(0.2);
      behaviorMessager.registerTopicListener(FootstepPlanForUI, footsteps ->
      {
         reviewingBodyPath = false;
         footstepPlanGraphic.generateMeshesAsynchronously(footsteps);
      });
      commandedFootsteps = new FootstepPlanWithTextGraphic();
      behaviorMessager.registerTopicListener(LastCommandedFootsteps, commandedFootsteps::generateMeshesAsynchronously);

      planarRegionsGraphic = new LivePlanarRegionsGraphic(false);
      behaviorMessager.registerTopicListener(PlanarRegionsForUI, planarRegions -> {
         planarRegionsGraphic.acceptPlanarRegions(planarRegions);
      });

      goalGraphic = new PoseGraphic("Goal", Color.DEEPSKYBLUE, 0.03);
      new IHMCROS2Callback<>(ros2Node, GOAL_INPUT, goal ->
      {
         Platform.runLater(() -> goalGraphic.setPose(goal));
      });

      closestPointAlongPathGraphic = new PoseGraphic("Closest", Color.BLUE, 0.027);
      behaviorMessager.registerTopicListener(ClosestPointForUI, pose -> Platform.runLater(() -> closestPointAlongPathGraphic.setPose(pose)));
      subGoalGraphic = new PoseGraphic("Sub goal", Color.YELLOW, 0.027);
      behaviorMessager.registerTopicListener(SubGoalForUI, pose -> Platform.runLater(() -> subGoalGraphic.setPose(pose)));

      bodyPathPlanGraphic = new BodyPathPlanGraphic();
      behaviorMessager.registerTopicListener(BodyPathPlanForUI, bodyPathPlan ->
      {
         reviewingBodyPath = true;
         ArrayList<Point3DReadOnly> bodyPathAsPoints = new ArrayList<>();
         for (Pose3D pose3D : bodyPathPlan)
         {
            bodyPathAsPoints.add(pose3D.getPosition());
         }
         Platform.runLater(() -> bodyPathPlanGraphic.generateMeshesAsynchronously(bodyPathAsPoints));
      });
   }

   public void setEnabled(boolean enabled)
   {
      if (!enabled)
      {
         clearGraphics();
         Platform.runLater(() -> getChildren().remove(closestPointAlongPathGraphic));
         Platform.runLater(() -> getChildren().remove(subGoalGraphic));
         Platform.runLater(() -> getChildren().remove(goalGraphic));
         Platform.runLater(() -> getChildren().remove(bodyPathPlanGraphic));
         Platform.runLater(() -> getChildren().remove(planarRegionsGraphic));
         Platform.runLater(() -> getChildren().remove(startAndGoalFootPoses));
         Platform.runLater(() -> getChildren().remove(footstepPlanGraphic));
         Platform.runLater(() -> getChildren().remove(commandedFootsteps));
      }
      else
      {
         Platform.runLater(() -> getChildren().add(closestPointAlongPathGraphic));
         Platform.runLater(() -> getChildren().add(subGoalGraphic));
         Platform.runLater(() -> getChildren().add(goalGraphic));
         Platform.runLater(() -> getChildren().add(bodyPathPlanGraphic));
         Platform.runLater(() -> getChildren().add(planarRegionsGraphic));
         Platform.runLater(() -> getChildren().add(startAndGoalFootPoses));
         Platform.runLater(() -> getChildren().add(footstepPlanGraphic));
         Platform.runLater(() -> getChildren().add(commandedFootsteps));
      }
   }

   public boolean isReviewingBodyPath()
   {
      return reviewingBodyPath;
   }

   public void clearBodyPathPlanGraphic()
   {
      bodyPathPlanGraphic.clear();
   }

   public void clearFootstepPlanGraphic()
   {
      footstepPlanGraphic.clear();
   }

   public void clearGraphics()
   {
      planarRegionsGraphic.clear();
      bodyPathPlanGraphic.clear();
      startAndGoalFootPoses.clear();
      footstepPlanGraphic.clear();
      commandedFootsteps.clear();
      closestPointAlongPathGraphic.clear();
   }

   public void destroy()
   {
      planarRegionsGraphic.destroy();
      footstepPlanGraphic.destroy();
      commandedFootsteps.destroy();
      startAndGoalFootPoses.destroy();
      bodyPathPlanGraphic.destroy();
   }
}
