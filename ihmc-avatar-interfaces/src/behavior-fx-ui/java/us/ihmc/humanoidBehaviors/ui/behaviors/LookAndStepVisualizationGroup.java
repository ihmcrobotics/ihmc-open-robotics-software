package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.humanoidBehaviors.tools.ManagedMessager;
import us.ihmc.humanoidBehaviors.tools.ros2.ManagedROS2Node;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.JavaFXLivePlanarRegionsGraphic;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.BodyPathPlanForUI;

public class LookAndStepVisualizationGroup extends Group
{
   private final ManagedROS2Node ros2Node;
   private final ManagedMessager messager;

   private final FootstepPlanGraphic footstepPlanGraphic;
   private final FootstepPlanGraphic commandedFootsteps;
   private final FootstepPlanGraphic startAndGoalFootPoses;
   private final PoseGraphic closestPointAlongPathGraphic;
   private final PoseGraphic subGoalGraphic;
   private final BodyPathPlanGraphic bodyPathPlanGraphic;
   private final JavaFXLivePlanarRegionsGraphic planarRegionsGraphic;
   private final PoseGraphic goalGraphic;

   private boolean reviewingBodyPath = true;

   public LookAndStepVisualizationGroup(ROS2NodeInterface parentROS2Node, Messager parentMessager)
   {
      ros2Node = new ManagedROS2Node(parentROS2Node);
      messager = new ManagedMessager(parentMessager);

      startAndGoalFootPoses = new FootstepPlanGraphic();
      startAndGoalFootPoses.setColor(RobotSide.LEFT, Color.BLUE);
      startAndGoalFootPoses.setColor(RobotSide.RIGHT, Color.BLUE);
      startAndGoalFootPoses.setTransparency(0.4);
      messager.registerTopicListener(StartAndGoalFootPosesForUI, startAndGoalFootPoses::generateMeshesAsynchronously);
      footstepPlanGraphic = new FootstepPlanGraphic();
      footstepPlanGraphic.setTransparency(0.2);
      messager.registerTopicListener(FootstepPlanForUI, footsteps ->
      {
         reviewingBodyPath = false;
         footstepPlanGraphic.generateMeshesAsynchronously(footsteps);
      });
      commandedFootsteps = new FootstepPlanGraphic();
      messager.registerTopicListener(LastCommandedFootsteps, commandedFootsteps::generateMeshesAsynchronously);

      planarRegionsGraphic = new JavaFXLivePlanarRegionsGraphic(false);
      messager.registerTopicListener(PlanarRegionsForUI, planarRegionsGraphic::acceptPlanarRegions);

      goalGraphic = new PoseGraphic("Goal", Color.DEEPSKYBLUE, 0.03);
      new IHMCROS2Callback<>(ros2Node, GOAL_INPUT, goal ->
      {
         Platform.runLater(() -> goalGraphic.setPose(goal));
      });

      closestPointAlongPathGraphic = new PoseGraphic("Closest", Color.BLUE, 0.027);
      messager.registerTopicListener(ClosestPointForUI, pose -> Platform.runLater(() -> closestPointAlongPathGraphic.setPose(pose)));
      subGoalGraphic = new PoseGraphic("Sub goal", Color.YELLOW, 0.027);
      messager.registerTopicListener(SubGoalForUI, pose -> Platform.runLater(() -> subGoalGraphic.setPose(pose)));

      bodyPathPlanGraphic = new BodyPathPlanGraphic();
      messager.registerTopicListener(BodyPathPlanForUI, bodyPathPlan ->
      {
         reviewingBodyPath = true;
         Platform.runLater(() -> bodyPathPlanGraphic.generateMeshesAsynchronously(bodyPathPlan));
      });
   }

   public void setEnabled(boolean enabled)
   {
      ros2Node.setEnabled(enabled);
      messager.setEnabled(enabled);
      if (!enabled)
      {
         clearGraphics(); // Do we always want to clear?
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
