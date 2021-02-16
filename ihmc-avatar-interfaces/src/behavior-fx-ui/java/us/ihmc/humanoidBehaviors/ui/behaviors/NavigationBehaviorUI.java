package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.layout.Pane;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.navigation.NavigationBehavior;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.JavaFXLivePlanarRegionsGraphic;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

import static us.ihmc.humanoidBehaviors.navigation.NavigationBehavior.NavigationBehaviorAPI.*;

public class NavigationBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(NavigationBehavior.DEFINITION, NavigationBehaviorUI::new);

   private final FootstepPlanGraphic footstepPlanGraphic;
   private final BodyPathPlanGraphic bodyPathPlanGraphic;

   public NavigationBehaviorUI(SubScene sceneNode, Pane visualizationPane, ROS2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      super(sceneNode, visualizationPane, ros2Node, behaviorMessager, robotModel);

      footstepPlanGraphic = new FootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      get3DGroup().getChildren().add(footstepPlanGraphic);
      behaviorMessager.registerTopicListener(FootstepPlanForUI, footstepPlan ->
            footstepPlanGraphic.generateMeshesAsynchronously(MinimalFootstep.convertPairListToMinimalFoostepList(footstepPlan)));

      bodyPathPlanGraphic = new BodyPathPlanGraphic();
      get3DGroup().getChildren().add(bodyPathPlanGraphic);
      behaviorMessager.registerTopicListener(BodyPathPlanForUI, bodyPathPlanGraphic::generateMeshesAsynchronously);

      JavaFXLivePlanarRegionsGraphic livePlanarRegionsGraphic = new JavaFXLivePlanarRegionsGraphic(false);
      get3DGroup().getChildren().add(livePlanarRegionsGraphic);
      behaviorMessager.registerTopicListener(MapRegionsForUI, livePlanarRegionsGraphic::acceptPlanarRegions);
   }

   @Override
   public void setEnabled(boolean enabled)
   {

   }

   @FXML public void step()
   {
      getBehaviorMessager().submitMessage(StepThroughAlgorithm, new Object());
   }

   @Override
   public void destroy()
   {

   }
}
