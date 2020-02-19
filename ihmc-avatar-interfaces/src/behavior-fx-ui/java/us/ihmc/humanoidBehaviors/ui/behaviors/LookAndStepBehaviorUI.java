package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.control.TableView;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyTable;
import us.ihmc.messager.Messager;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.*;

public class LookAndStepBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(LookAndStepBehavior.DEFINITION, LookAndStepBehaviorUI::new);

   private final LookAndStepBehaviorParameters parameters = new LookAndStepBehaviorParameters();

   private Messager behaviorMessager;
   private FootstepPlanGraphic footstepPlanGraphic;

   @FXML private TableView parameterTable;

   @Override
   public void init(SubScene sceneNode, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.behaviorMessager = behaviorMessager;

      footstepPlanGraphic = new FootstepPlanGraphic(robotModel);
      getChildren().add(footstepPlanGraphic);
      behaviorMessager.registerTopicListener(FootstepPlanForUI, footstepPlanGraphic::generateMeshesAsynchronously);

      LivePlanarRegionsGraphic livePlanarRegionsGraphic = new LivePlanarRegionsGraphic(false);
      getChildren().add(livePlanarRegionsGraphic);
      behaviorMessager.registerTopicListener(MapRegionsForUI, livePlanarRegionsGraphic::acceptPlanarRegions);

      JavaFXStoredPropertyTable javaFXStoredPropertyTable = new JavaFXStoredPropertyTable(parameterTable);
      javaFXStoredPropertyTable.setup(parameters, LookAndStepBehaviorParameters.keys, this::publishParameters);
   }

   @Override
   public void setEnabled(boolean enabled)
   {

   }

   private void publishParameters()
   {
      behaviorMessager.submitMessage(Parameters, parameters.getAllAsStrings());
   }

   @FXML public void takeStep()
   {
      behaviorMessager.submitMessage(TakeStep, new Object());
   }
}
