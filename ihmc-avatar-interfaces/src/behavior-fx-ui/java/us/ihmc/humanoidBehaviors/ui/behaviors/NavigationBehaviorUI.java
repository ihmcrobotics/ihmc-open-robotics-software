package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.layout.Pane;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.navigation.NavigationBehavior;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2NodeInterface;

import static us.ihmc.humanoidBehaviors.navigation.NavigationBehavior.NavigationBehaviorAPI.*;

public class NavigationBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(NavigationBehavior.DEFINITION, NavigationBehaviorUI::new);

   private Messager behaviorMessager;
   private FootstepPlanGraphic footstepPlanGraphic;
   private BodyPathPlanGraphic bodyPathPlanGraphic;

   @Override
   public void init(SubScene sceneNode, Pane visualizationPane, Ros2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.behaviorMessager = behaviorMessager;

      footstepPlanGraphic = new FootstepPlanGraphic(robotModel);
      getChildren().add(footstepPlanGraphic);
      behaviorMessager.registerTopicListener(FootstepPlanForUI, footstepPlanGraphic::generateMeshesAsynchronously);

      bodyPathPlanGraphic = new BodyPathPlanGraphic();
      getChildren().add(bodyPathPlanGraphic);
      behaviorMessager.registerTopicListener(BodyPathPlanForUI, bodyPathPlanGraphic::generateMeshesAsynchronously);

      LivePlanarRegionsGraphic livePlanarRegionsGraphic = new LivePlanarRegionsGraphic(false);
      getChildren().add(livePlanarRegionsGraphic);
      behaviorMessager.registerTopicListener(MapRegionsForUI, livePlanarRegionsGraphic::acceptPlanarRegions);
   }

   @Override
   public void setEnabled(boolean enabled)
   {

   }

   @FXML public void step()
   {
      behaviorMessager.submitMessage(StepThroughAlgorithm, new Object());
   }
}
