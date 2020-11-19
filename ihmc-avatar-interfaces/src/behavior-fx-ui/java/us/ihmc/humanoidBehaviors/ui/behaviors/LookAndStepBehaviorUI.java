package us.ihmc.humanoidBehaviors.ui.behaviors;

import controller_msgs.msg.dds.StoredPropertySetMessage;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.humanoidBehaviors.tools.ros2.ROS2PublisherMap;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.humanoidBehaviors.ui.editors.OrientationYawEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor.EditMode;
import us.ihmc.humanoidBehaviors.ui.model.FXUIActionMap;
import us.ihmc.humanoidBehaviors.ui.model.FXUITrigger;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyTable;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(LookAndStepBehavior.DEFINITION, LookAndStepBehaviorUI::new);

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();
   private FootstepPlannerParametersBasics footstepPlannerParameters;
   private SwingPlannerParametersBasics swingPlannerParameters;

   private Messager behaviorMessager;
   private ROS2PublisherMap ros2Publisher;
   private LookAndStepVisualizationGroup lookAndStepVisualizationGroup;
   private PoseGraphic goalGraphic;

   private SnappedPositionEditor snappedPositionEditor;
   private OrientationYawEditor orientationYawEditor;

   private FXUIActionMap placeGoalActionMap;

   @FXML private Button placeGoalButton;
   @FXML private Button publishSupportRegions;
   @FXML private CheckBox operatorReviewCheckBox;
   @FXML private TextField behaviorState;
   @FXML private TableView lookAndStepParameterTable;
   @FXML private TableView footstepPlannerParameterTable;
   @FXML private TableView swingPlannerParameterTable;

   @Override
   public void init(SubScene sceneNode, Pane visualizationPane, ROS2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.behaviorMessager = behaviorMessager;
      ros2Publisher = new ROS2PublisherMap(ros2Node);

      View3DFactory view2DFactory = View3DFactory.createSubscene(false, SceneAntialiasing.BALANCED);
      view2DFactory.addCameraController(0.05, 2000.0, true);
      view2DFactory.setBackgroundColor(Color.WHITE);
      Group view2DGroup = view2DFactory.getRoot();
      Pane view2DPane = view2DFactory.getSubSceneWrappedInsidePane();
      // TODO: Find out why the the view2D can't be added to the VBox

      Rectangle rectangle = new Rectangle(0.0, 0.0, 50.0, 50.0);
      rectangle.setFill(Color.BLUE);
      view2DGroup.getChildren().add(rectangle);

      lookAndStepVisualizationGroup = new LookAndStepVisualizationGroup(ros2Node, behaviorMessager);
      getChildren().add(lookAndStepVisualizationGroup);
      goalGraphic = new PoseGraphic("Goal", Color.CADETBLUE, 0.03);

      JavaFXStoredPropertyTable lookAndStepJavaFXStoredPropertyTable = new JavaFXStoredPropertyTable(lookAndStepParameterTable);
      lookAndStepJavaFXStoredPropertyTable.setup(lookAndStepParameters, LookAndStepBehaviorParameters.keys, this::publishLookAndStepParameters);

      footstepPlannerParameters = robotModel.getFootstepPlannerParameters("ForLookAndStep");
      JavaFXStoredPropertyTable footstepPlannerJavaFXStoredPropertyTable = new JavaFXStoredPropertyTable(footstepPlannerParameterTable);
      footstepPlannerJavaFXStoredPropertyTable.setup(footstepPlannerParameters, FootstepPlannerParameterKeys.keys, this::publishFootstepPlanningParameters);

      swingPlannerParameters = robotModel.getSwingPlannerParameters("ForLookAndStep");
      JavaFXStoredPropertyTable swingPlannerJavaFXStoredPropertyTable = new JavaFXStoredPropertyTable(swingPlannerParameterTable);
      swingPlannerJavaFXStoredPropertyTable.setup(swingPlannerParameters, SwingPlannerParameterKeys.keys, this::publishSwingPlanningParameters);

      behaviorMessager.registerTopicListener(CurrentState, state -> Platform.runLater(() -> behaviorState.setText(state)));
      behaviorMessager.registerTopicListener(OperatorReviewEnabledToUI, enabled -> Platform.runLater(() -> operatorReviewCheckBox.setSelected(enabled)));

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
         ros2Publisher.publish(GOAL_INPUT, new Pose3D(goalGraphic.getPose()));

         placeGoalButton.setDisable(false);
      });
      placeGoalActionMap.mapAction(FXUITrigger.RIGHT_CLICK, trigger ->
      {
         placeGoalButton.setDisable(false);
      });

      behaviorMessager.registerTopicListener(ResetForUI, message -> lookAndStepVisualizationGroup.clearGraphics());

      // TODO Add joystick support
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      lookAndStepVisualizationGroup.setEnabled(enabled);
      if (!enabled)
      {
         Platform.runLater(() -> getChildren().remove(goalGraphic));
      }
      else
      {
         Platform.runLater(() -> getChildren().add(goalGraphic));
      }
   }

   private void publishLookAndStepParameters()
   {
      StoredPropertySetMessage storedPropertySetMessage = new StoredPropertySetMessage();
      lookAndStepParameters.getAllAsStrings().forEach(value -> storedPropertySetMessage.getStrings().add(value));
      ros2Publisher.publish(LOOK_AND_STEP_PARAMETERS, storedPropertySetMessage);
   }

   private void publishFootstepPlanningParameters()
   {
      behaviorMessager.submitMessage(FootstepPlannerParameters, footstepPlannerParameters.getAllAsStrings());
   }

   private void publishSwingPlanningParameters()
   {
      behaviorMessager.submitMessage(SwingPlannerParameters, swingPlannerParameters.getAllAsStrings());
   }

   @FXML public void placeGoalButton()
   {
      placeGoalActionMap.start();
   }

   @FXML public void approve()
   {
      behaviorMessager.submitMessage(ReviewApproval, true);
   }

   @FXML public void reject()
   {
      behaviorMessager.submitMessage(ReviewApproval, false);
      if (lookAndStepVisualizationGroup.isReviewingBodyPath())
         lookAndStepVisualizationGroup.clearBodyPathPlanGraphic();
      else
         lookAndStepVisualizationGroup.clearFootstepPlanGraphic();
   }

   @FXML public void saveLookAndStepParameters()
   {
      lookAndStepParameters.save();
   }

   @FXML public void saveFootstepPlanningParameters()
   {
      footstepPlannerParameters.save();
   }

   @FXML public void saveSwingPlannerParameters()
   {
      swingPlannerParameters.save();
   }

   @FXML public void publishSupportRegions()
   {
      behaviorMessager.submitMessage(PublishSupportRegions, new Object());
   }

   @FXML public void operatorReviewCheckBox()
   {
      behaviorMessager.submitMessage(OperatorReviewEnabled, operatorReviewCheckBox.isSelected());
   }

   @FXML public void reset()
   {
      ros2Publisher.publish(RESET);
   }

   @Override
   public void destroy()
   {
      lookAndStepVisualizationGroup.destroy();
   }
}
