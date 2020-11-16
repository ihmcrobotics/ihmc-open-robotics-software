package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.paint.Color;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanWithTextGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.messager.Messager;

import java.util.ArrayList;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.BodyPathPlanForUI;

public class LookAndStepVisualizationGroup extends Group
{
   private FootstepPlanWithTextGraphic footstepPlanGraphic;
   private FootstepPlanWithTextGraphic startAndGoalFootPoses;
   private PoseGraphic closestPointAlongPathGraphic;
   private PoseGraphic subGoalGraphic;
   private BodyPathPlanGraphic bodyPathPlanGraphic;
   private LivePlanarRegionsGraphic planarRegionsGraphic;
   private PoseGraphic goalGraphic;

   public LookAndStepVisualizationGroup(Messager behaviorMessager)
   {
      startAndGoalFootPoses = new FootstepPlanWithTextGraphic();
      behaviorMessager.registerTopicListener(StartAndGoalFootPosesForUI, startAndGoalFootPoses::generateMeshesAsynchronously);
      footstepPlanGraphic = new FootstepPlanWithTextGraphic();
      behaviorMessager.registerTopicListener(FootstepPlanForUI, footstepPlanGraphic::generateMeshesAsynchronously);

      planarRegionsGraphic = new LivePlanarRegionsGraphic(false);
      behaviorMessager.registerTopicListener(PlanarRegionsForUI, planarRegions -> {
         planarRegionsGraphic.acceptPlanarRegions(planarRegions);
      });

      goalGraphic = new PoseGraphic("Goal", Color.CADETBLUE, 0.03);

      closestPointAlongPathGraphic = new PoseGraphic("Closest", Color.BLUE, 0.027);
      behaviorMessager.registerTopicListener(ClosestPointForUI, pose -> Platform.runLater(() -> closestPointAlongPathGraphic.setPose(pose)));
      subGoalGraphic = new PoseGraphic("Sub goal", Color.YELLOW, 0.027);
      behaviorMessager.registerTopicListener(SubGoalForUI, pose -> Platform.runLater(() -> subGoalGraphic.setPose(pose)));

      bodyPathPlanGraphic = new BodyPathPlanGraphic();
      behaviorMessager.registerTopicListener(BodyPathPlanForUI, bodyPathPlan ->
      {
         ArrayList<Point3DReadOnly> bodyPathAsPoints = new ArrayList<>();
         for (Pose3D pose3D : bodyPathPlan)
         {
            bodyPathAsPoints.add(pose3D.getPosition());
         }
         Platform.runLater(() -> bodyPathPlanGraphic.generateMeshesAsynchronously(bodyPathAsPoints));
      });

      getChildren().add(footstepPlanGraphic);
      getChildren().add(bodyPathPlanGraphic);
      getChildren().add(closestPointAlongPathGraphic);
      getChildren().add(subGoalGraphic);
      getChildren().add(goalGraphic);
      getChildren().add(planarRegionsGraphic);
      getChildren().add(startAndGoalFootPoses);
   }
}
