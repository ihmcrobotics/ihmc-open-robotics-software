package us.ihmc.behaviors.javafx.behaviors;

import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.layout.Pane;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.javafx.JavaFXBehaviorUIDefinition;
import us.ihmc.behaviors.javafx.JavaFXBehaviorUIInterface;
import us.ihmc.behaviors.javafx.graphics.BodyPathPlanGraphic;
import us.ihmc.behaviors.navigation.NavigationBehavior;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.behaviors.javafx.graphics.FootstepPlanGraphic;
import us.ihmc.behaviors.javafx.graphics.live.JavaFXLivePlanarRegionsGraphic;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

import static us.ihmc.behaviors.navigation.NavigationBehavior.NavigationBehaviorAPI.*;

public class NavigationBehaviorUI extends JavaFXBehaviorUIInterface
{
   public static final JavaFXBehaviorUIDefinition DEFINITION = new JavaFXBehaviorUIDefinition(NavigationBehavior.DEFINITION, NavigationBehaviorUI::new);

   private final FootstepPlanGraphic footstepPlanGraphic;
   private final BodyPathPlanGraphic bodyPathPlanGraphic;

   public NavigationBehaviorUI(SubScene sceneNode, Pane visualizationPane, ROS2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      super(sceneNode, visualizationPane, ros2Node, behaviorMessager, robotModel);

      footstepPlanGraphic = new FootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      get3DGroup().getChildren().add(footstepPlanGraphic);
      behaviorMessager.registerTopicListener(FootstepPlanForUI, footstepPlan ->
            footstepPlanGraphic.generateMeshesAsynchronously(MinimalFootstep.convertPairListToMinimalFoostepList(footstepPlan, DEFINITION.getName())));

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
