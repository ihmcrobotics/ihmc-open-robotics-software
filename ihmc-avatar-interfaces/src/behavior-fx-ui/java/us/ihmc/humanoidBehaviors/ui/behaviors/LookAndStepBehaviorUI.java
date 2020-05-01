package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.paint.Color;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.humanoidBehaviors.ui.editors.OrientationYawEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.model.FXUIActionMap;
import us.ihmc.humanoidBehaviors.ui.model.FXUITrigger;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyTable;
import us.ihmc.messager.Messager;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.*;

public class LookAndStepBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(LookAndStepBehavior.DEFINITION, LookAndStepBehaviorUI::new);

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();
   private FootstepPlannerParametersBasics footstepPlannerParameters;

   private Messager behaviorMessager;
   private FootstepPlanGraphic footstepPlanGraphic;
   private LivePlanarRegionsGraphic livePlanarRegionsGraphic;
   private PositionGraphic subGoalGraphic;
   private BodyPathPlanGraphic bodyPathPlanGraphic;
   private PoseGraphic goalGraphic = new PoseGraphic("Goal", Color.CADETBLUE, 0.03);

   private SnappedPositionEditor snappedPositionEditor;
   private OrientationYawEditor orientationYawEditor;

   private FXUIActionMap placeGoalActionMap;

   @FXML private Button placeGoalButton;
   @FXML private CheckBox operatorReviewCheckBox;
   @FXML private TextField behaviorState;
   @FXML private TableView lookAndStepParameterTable;
   @FXML private TableView footstepPlannerParameterTable;

   @Override
   public void init(SubScene sceneNode, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.behaviorMessager = behaviorMessager;

      footstepPlanGraphic = new FootstepPlanGraphic(robotModel);
      getChildren().add(footstepPlanGraphic);
      behaviorMessager.registerTopicListener(FootstepPlanForUI, footstepPlanGraphic::generateMeshesAsynchronously);

      livePlanarRegionsGraphic = new LivePlanarRegionsGraphic(false);
      getChildren().add(livePlanarRegionsGraphic);
      behaviorMessager.registerTopicListener(MapRegionsForUI, livePlanarRegionsGraphic::acceptPlanarRegions);

      subGoalGraphic = new PositionGraphic(Color.GREEN, 0.02);
      behaviorMessager.registerTopicListener(SubGoalForUI, position -> Platform.runLater(() -> subGoalGraphic.setPosition(position)));

      bodyPathPlanGraphic = new BodyPathPlanGraphic();
      behaviorMessager.registerTopicListener(BodyPathPlanForUI,
                                             bodyPathPlan -> Platform.runLater(() -> bodyPathPlanGraphic.generateMeshesAsynchronously(bodyPathPlan)));

      JavaFXStoredPropertyTable lookAndStepJavaFXStoredPropertyTable = new JavaFXStoredPropertyTable(lookAndStepParameterTable);
      lookAndStepJavaFXStoredPropertyTable.setup(lookAndStepParameters, LookAndStepBehaviorParameters.keys, this::publishLookAndStepParameters);

      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      JavaFXStoredPropertyTable footstepPlannerJavaFXStoredPropertyTable = new JavaFXStoredPropertyTable(footstepPlannerParameterTable);
      footstepPlannerJavaFXStoredPropertyTable.setup(footstepPlannerParameters, FootstepPlannerParameterKeys.keys, this::footstepPlanningParameters);

      behaviorMessager.registerTopicListener(CurrentState, state -> Platform.runLater(() -> behaviorState.setText(state)));

      snappedPositionEditor = new SnappedPositionEditor(sceneNode);
      orientationYawEditor = new OrientationYawEditor(sceneNode);

      placeGoalActionMap = new FXUIActionMap(startAction ->
      {
         placeGoalButton.setDisable(true);
         goalGraphic.setVisible(false);
         snappedPositionEditor.edit(SnappedPositionEditor.EditMode.XY_PLANE, goalGraphic, exitType ->
         {
            placeGoalActionMap.triggerAction(exitType);
         });
      });
      placeGoalActionMap.mapAction(FXUITrigger.POSITION_LEFT_CLICK, trigger ->
      {
         goalGraphic.setVisible(true);
         orientationYawEditor.edit(goalGraphic, exitType -> placeGoalActionMap.triggerAction(exitType));
      });
      placeGoalActionMap.mapAction(FXUITrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         // calculate and send body path plan

         placeGoalButton.setDisable(false);
      });

//      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, this::mouseClicked);

      // TODO Add joystick support
   }

//   private final void mouseClicked(MouseEvent event)
//   {
//      if (!event.isConsumed() && event.isStillSincePress() && BehaviorUI.ACTIVE_EDITOR == null)
//      {
//         PickResult pickResult = event.getPickResult();
//         Node intersectedNode = pickResult.getIntersectedNode();
//      }
//   }

   @Override
   public void setEnabled(boolean enabled)
   {
      if (!enabled)
      {
         livePlanarRegionsGraphic.clear();
         footstepPlanGraphic.clear();
         Platform.runLater(() -> getChildren().remove(subGoalGraphic.getNode()));
      }
      else
      {
         Platform.runLater(() -> getChildren().add(subGoalGraphic.getNode()));
      }
   }

   private void publishLookAndStepParameters()
   {
      behaviorMessager.submitMessage(LookAndStepParameters, lookAndStepParameters.getAllAsStrings());
   }

   private void footstepPlanningParameters()
   {
      behaviorMessager.submitMessage(FootstepPlannerParameters, footstepPlannerParameters.getAllAsStrings());
   }

   @FXML public void placeGoalButton()
   {

   }

   @FXML public void takeStep()
   {
      behaviorMessager.submitMessage(TakeStep, new Object());
   }

   @FXML public void rePlan()
   {
      behaviorMessager.submitMessage(RePlan, new Object());
   }

   @FXML public void saveLookAndStepParameters()
   {
      lookAndStepParameters.save();
   }

   @FXML public void saveFootstepPlanningParameters()
   {
      footstepPlannerParameters.save();
   }

   @FXML public void operatorReviewCheckBox()
   {
      behaviorMessager.submitMessage(OperatorReviewEnabled, operatorReviewCheckBox.isSelected());
   }
}
