package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.paint.Color;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.humanoidBehaviors.ui.editors.OrientationYawEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor.EditMode;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.model.FXUIActionMap;
import us.ihmc.humanoidBehaviors.ui.model.FXUITrigger;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyTable;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2NodeInterface;

import java.util.ArrayList;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.*;

public class LookAndStepBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(LookAndStepBehavior.DEFINITION, LookAndStepBehaviorUI::new);

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();
   private FootstepPlannerParametersBasics footstepPlannerParameters;

   private Messager behaviorMessager;
   private FootstepPlanGraphic footstepPlanGraphic;
   private LivePlanarRegionsGraphic livePlanarRegionsGraphic;
   private PoseGraphic subGoalGraphic;
   private BodyPathPlanGraphic bodyPathPlanGraphic;
   private PoseGraphic goalGraphic;
   
   private SnappedPositionEditor snappedPositionEditor;
   private OrientationYawEditor orientationYawEditor;

   private FXUIActionMap placeGoalActionMap;

   @FXML private Button placeGoalButton;
   @FXML private CheckBox operatorReviewCheckBox;
   @FXML private TextField behaviorState;
   @FXML private TableView lookAndStepParameterTable;
   @FXML private TableView footstepPlannerParameterTable;

   @Override
   public void init(SubScene sceneNode, Ros2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.behaviorMessager = behaviorMessager;

      footstepPlanGraphic = new FootstepPlanGraphic(robotModel);
      behaviorMessager.registerTopicListener(FootstepPlanForUI, footstepPlanGraphic::generateMeshesAsynchronously);

      livePlanarRegionsGraphic = new LivePlanarRegionsGraphic(false);
      behaviorMessager.registerTopicListener(MapRegionsForUI, livePlanarRegionsGraphic::acceptPlanarRegions);

      goalGraphic = new PoseGraphic("Goal", Color.CADETBLUE, 0.03);

      subGoalGraphic = new PoseGraphic("Sub goal", Color.YELLOW, 0.027);
      behaviorMessager.registerTopicListener(SubGoalForUI, pose -> Platform.runLater(() -> subGoalGraphic.setPose(pose)));

      bodyPathPlanGraphic = new BodyPathPlanGraphic();
      behaviorMessager.registerTopicListener(BodyPathPlanForUI,
      bodyPathPlan ->
      {
         ArrayList<Point3DReadOnly> bodyPathAsPoints = new ArrayList<>();
         for (Pose3D pose3D : bodyPathPlan)
         {
            bodyPathAsPoints.add(pose3D.getPosition());
         }
         Platform.runLater(() -> bodyPathPlanGraphic.generateMeshesAsynchronously(bodyPathAsPoints));
      });

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
         snappedPositionEditor.edit(EditMode.BOTH, goalGraphic, exitType ->
         {
            placeGoalActionMap.triggerAction(exitType);
         });
      });
      placeGoalActionMap.mapAction(FXUITrigger.POSITION_LEFT_CLICK, trigger ->
      {
         orientationYawEditor.edit(goalGraphic, exitType -> placeGoalActionMap.triggerAction(exitType));
      });
      placeGoalActionMap.mapAction(FXUITrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         behaviorMessager.submitMessage(GoalInput, new Pose3D(goalGraphic.getPose()));

         placeGoalButton.setDisable(false);
      });
      placeGoalActionMap.mapAction(FXUITrigger.RIGHT_CLICK, trigger ->
      {
         placeGoalButton.setDisable(false);
      });

      // TODO Add joystick support
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      if (!enabled)
      {
         livePlanarRegionsGraphic.clear();
         footstepPlanGraphic.clear();
         Platform.runLater(() -> getChildren().remove(subGoalGraphic));
         Platform.runLater(() -> getChildren().remove(goalGraphic));
         Platform.runLater(() -> getChildren().remove(bodyPathPlanGraphic));
         Platform.runLater(() -> getChildren().remove(livePlanarRegionsGraphic));
         Platform.runLater(() -> getChildren().remove(footstepPlanGraphic));
      }
      else
      {
         Platform.runLater(() -> getChildren().add(subGoalGraphic));
         Platform.runLater(() -> getChildren().add(goalGraphic));
         Platform.runLater(() -> getChildren().add(bodyPathPlanGraphic));
         Platform.runLater(() -> getChildren().add(livePlanarRegionsGraphic));
         Platform.runLater(() -> getChildren().add(footstepPlanGraphic));
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
      placeGoalActionMap.start();
   }

   @FXML public void approve()
   {
//      behaviorMessager.submitMessage(TakeStep, new Object());
      behaviorMessager.submitMessage(Approval, true);
   }

   @FXML public void reject()
   {
//      behaviorMessager.submitMessage(RePlan, new Object());
      behaviorMessager.submitMessage(Approval, false);
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
