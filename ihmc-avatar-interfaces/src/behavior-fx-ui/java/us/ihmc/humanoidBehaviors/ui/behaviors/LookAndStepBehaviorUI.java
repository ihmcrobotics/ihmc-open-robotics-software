package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.ScatterChart;
import javafx.scene.chart.XYChart;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.humanoidBehaviors.tools.ros2.ROS2PublisherMap;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.humanoidBehaviors.ui.editors.OrientationYawEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor.EditMode;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanWithTextGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.model.FXUIActionMap;
import us.ihmc.humanoidBehaviors.ui.model.FXUITrigger;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyTable;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2NodeInterface;

import java.util.ArrayList;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class LookAndStepBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(LookAndStepBehavior.DEFINITION, LookAndStepBehaviorUI::new);

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();
   private FootstepPlannerParametersBasics footstepPlannerParameters;

   private Messager behaviorMessager;
   private ROS2PublisherMap ros2Publisher;
   private FootstepPlanWithTextGraphic footstepPlanGraphic;
   private FootstepPlanWithTextGraphic startAndGoalFootPoses;
   private LivePlanarRegionsGraphic planarRegionsRegionsGraphic;
   private PoseGraphic closestPointAlongPathGraphic;
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
   public void init(SubScene sceneNode, Pane visualizationPane, Ros2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
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

      NumberAxis xAxis = new NumberAxis(0.0, 10.0, 1.0);
      NumberAxis yAxis = new NumberAxis(0.0, 10.0, 1.0);
      ScatterChart<Number, Number> scatterChart = new ScatterChart<>(xAxis, yAxis);

      XYChart.Series series1 = new XYChart.Series();
      series1.setName("Set1");
      series1.getData().add(new XYChart.Data(0.0, 3.0));
      series1.getData().add(new XYChart.Data(1.0, 5.0));
      series1.getData().add(new XYChart.Data(2.0, 3.0));
      series1.getData().add(new XYChart.Data(3.0, 3.0));
      series1.getData().add(new XYChart.Data(4.0, 3.0));
      series1.getData().add(new XYChart.Data(5.0, 3.0));
      series1.getData().add(new XYChart.Data(6.0, 3.0));
      series1.getData().add(new XYChart.Data(7.0, 3.0));
      series1.getData().add(new XYChart.Data(8.0, 3.0));
      series1.getData().add(new XYChart.Data(9.0, 3.0));
      series1.getData().add(new XYChart.Data(10.0, 3.0));

      scatterChart.getData().add(series1);

//      scatterChart.setPrefWidth(200.0);

      visualizationPane.getChildren().addAll(scatterChart);

      startAndGoalFootPoses = new FootstepPlanWithTextGraphic();
      behaviorMessager.registerTopicListener(StartAndGoalFootPosesForUI, startAndGoalFootPoses::generateMeshesAsynchronously);
      footstepPlanGraphic = new FootstepPlanWithTextGraphic();
      behaviorMessager.registerTopicListener(FootstepPlanForUI, footstepPlanGraphic::generateMeshesAsynchronously);

      planarRegionsRegionsGraphic = new LivePlanarRegionsGraphic(false);
      behaviorMessager.registerTopicListener(PlanarRegionsForUI, planarRegions -> {
         planarRegionsRegionsGraphic.acceptPlanarRegions(planarRegions);
      });

      goalGraphic = new PoseGraphic("Goal", Color.CADETBLUE, 0.03);

      closestPointAlongPathGraphic = new PoseGraphic("Closest", Color.BLUE, 0.027);
      behaviorMessager.registerTopicListener(ClosestPointForUI, pose -> Platform.runLater(() -> closestPointAlongPathGraphic.setPose(pose)));
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
         ros2Publisher.publish(GOAL_INPUT, new Pose3D(goalGraphic.getPose()));

         placeGoalButton.setDisable(false);
      });
      placeGoalActionMap.mapAction(FXUITrigger.RIGHT_CLICK, trigger ->
      {
         placeGoalButton.setDisable(false);
      });

      behaviorMessager.registerTopicListener(ResetForUI, message -> clearGraphics());

      // TODO Add joystick support
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      if (!enabled)
      {
         clearGraphics();
         Platform.runLater(() -> getChildren().remove(closestPointAlongPathGraphic));
         Platform.runLater(() -> getChildren().remove(subGoalGraphic));
         Platform.runLater(() -> getChildren().remove(goalGraphic));
         Platform.runLater(() -> getChildren().remove(bodyPathPlanGraphic));
         Platform.runLater(() -> getChildren().remove(planarRegionsRegionsGraphic));
         Platform.runLater(() -> getChildren().remove(startAndGoalFootPoses));
         Platform.runLater(() -> getChildren().remove(footstepPlanGraphic));
      }
      else
      {
         Platform.runLater(() -> getChildren().add(closestPointAlongPathGraphic));
         Platform.runLater(() -> getChildren().add(subGoalGraphic));
         Platform.runLater(() -> getChildren().add(goalGraphic));
         Platform.runLater(() -> getChildren().add(bodyPathPlanGraphic));
         Platform.runLater(() -> getChildren().add(planarRegionsRegionsGraphic));
         Platform.runLater(() -> getChildren().add(startAndGoalFootPoses));
         Platform.runLater(() -> getChildren().add(footstepPlanGraphic));
      }
   }

   private void clearGraphics()
   {
      planarRegionsRegionsGraphic.clear();
      startAndGoalFootPoses.clear();
      footstepPlanGraphic.clear();
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
      behaviorMessager.submitMessage(ReviewApproval, true);
   }

   @FXML public void reject()
   {
//      behaviorMessager.submitMessage(RePlan, new Object());
      behaviorMessager.submitMessage(ReviewApproval, false);
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

   @FXML public void reset()
   {
      ros2Publisher.publish(RESET);
   }
}
