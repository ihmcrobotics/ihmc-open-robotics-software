package us.ihmc.humanoidBehaviors.ui.behaviors.coordinator;

import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.text.Text;
import std_msgs.msg.dds.Empty;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationBehaviorAPI;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationBehavior;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationStateName;
import us.ihmc.humanoidBehaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepVisualizationGroup;
import us.ihmc.humanoidBehaviors.ui.editors.WalkingGoalPlacementEditor;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.ROS2NodeInterface;

import static us.ihmc.humanoidBehaviors.demo.BuildingExplorationBehaviorAPI.*;

public class BuildingExplorationBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(BuildingExplorationBehavior.DEFINITION,
                                                                                  BuildingExplorationBehaviorUI::new);

   @FXML private ComboBox<BuildingExplorationStateName> requestedState;
   @FXML private Spinner<Double> goalX;
   @FXML private Spinner<Double> goalY;
   @FXML private Spinner<Double> goalZ;
   @FXML private Text currentState;
   @FXML private Text debrisDetected;
   @FXML private Text stairsDetected;
   @FXML private Text doorDetected;
   @FXML private Button placeGoal;

   private final JavaFXRobotVisualizer robotVisualizer;
   private final LookAndStepVisualizationGroup lookAndStepVisualizationGroup;
   private final FootstepPlanGraphic footstepPlanGraphic;
   private final WalkingGoalPlacementEditor walkingGoalPlacementEditor = new WalkingGoalPlacementEditor();
   private final IHMCROS2Publisher<Empty> executeStairsStepsPublisher;
   private final IHMCROS2Publisher<Empty> replanStairsStepsPublisher;

   public BuildingExplorationBehaviorUI(SubScene subScene, Pane visualizationPane, ROS2NodeInterface ros2Node, Messager messager, DRCRobotModel robotModel)
   {
      super(subScene, visualizationPane, ros2Node, messager, robotModel);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    controller_msgs.msg.dds.RobotConfigurationData.class,
                                                    ControllerAPIDefinition.getOutputTopic(robotModel.getSimpleRobotName()),
                                                    s -> getBehaviorMessager().submitMessage(BuildingExplorationBehaviorAPI.RobotConfigurationData,
                                                                                             s.takeNextData()));

      robotVisualizer = new JavaFXRobotVisualizer(robotModel);
      lookAndStepVisualizationGroup = new LookAndStepVisualizationGroup(ros2Node, messager);
      lookAndStepVisualizationGroup.setEnabled(true);
      footstepPlanGraphic = new FootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      new IHMCROS2Callback<>(ros2Node, TraverseStairsBehaviorAPI.PLANNED_STEPS, footstepDataListMessage ->
            footstepPlanGraphic.generateMeshesAsynchronously(MinimalFootstep.convertFootstepDataListMessage(footstepDataListMessage)));

      robotVisualizer.start();

      PositionGraphic goalGraphic = new PositionGraphic(Color.GRAY, 0.05);
      get3DGroup().getChildren().add(goalGraphic.getNode());
      goalGraphic.setMouseTransparent(true);
      messager.registerTopicListener(Goal, newGoal -> goalGraphic.setPosition(newGoal.getPosition()));

      messager.registerTopicListener(RobotConfigurationData, robotVisualizer::submitNewConfiguration);
      get3DGroup().getChildren().add(robotVisualizer.getRootNode());
      get3DGroup().getChildren().add(lookAndStepVisualizationGroup);
      get3DGroup().getChildren().add(footstepPlanGraphic);

      requestedState.setItems(FXCollections.observableArrayList(BuildingExplorationStateName.values()));

      goalX.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      goalY.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      goalZ.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));

      goalZ.valueProperty().addListener((observable, oldValue, newValue) ->
      {
         Point3D updatedPosition = new Point3D(goalGraphic.getPose().getPosition());
         updatedPosition.setZ(newValue);
         goalGraphic.setPosition(updatedPosition);
         goalGraphic.update();
      });

      goalX.getValueFactory().setValue(14.2);
      goalZ.getValueFactory().setValue(0.86);

      currentState.setText("----");
      debrisDetected.setText("No");
      stairsDetected.setText("No");
      doorDetected.setText("No");

      messager.registerTopicListener(CurrentState, state ->
      {
         currentState.setText(state.toString());
         if (state == BuildingExplorationStateName.LOOK_AND_STEP)
         {
            debrisDetected.setText("No");
            stairsDetected.setText("No");
         }

         if (state != BuildingExplorationStateName.WALK_THROUGH_DOOR)
         {
            doorDetected.setText("No");
         }
      });

      messager.registerTopicListener(DebrisDetected, d -> debrisDetected.setText("Yes"));
      messager.registerTopicListener(StairsDetected, d -> stairsDetected.setText("Yes"));
      messager.registerTopicListener(DoorDetected, d -> doorDetected.setText("Yes"));

      requestedState.getSelectionModel()
                    .selectedItemProperty()
                    .addListener((observable, oldState, newState) -> messager.submitMessage(BuildingExplorationBehaviorAPI.RequestedState, newState));

      executeStairsStepsPublisher = new IHMCROS2Publisher<>(ros2Node, TraverseStairsBehaviorAPI.EXECUTE_STEPS);
      replanStairsStepsPublisher = new IHMCROS2Publisher<>(ros2Node, TraverseStairsBehaviorAPI.REPLAN);

      walkingGoalPlacementEditor.init(subScene, placeGoal, placedGoal -> messager.submitMessage(BuildingExplorationBehaviorAPI.Goal, placedGoal));
      get3DGroup().getChildren().add(walkingGoalPlacementEditor);
   }

   @Override
   public void setEnabled(boolean enabled)
   {

   }

   @FXML
   public void requestStart()
   {
      getBehaviorMessager().submitMessage(BuildingExplorationBehaviorAPI.Start, true);
   }

   @FXML
   public void requestStop()
   {
      getBehaviorMessager().submitMessage(BuildingExplorationBehaviorAPI.Stop, true);
   }

   @FXML
   public void placeGoal()
   {
      walkingGoalPlacementEditor.startGoalPlacement();
   }

   @FXML
   public void ignoreDebris()
   {
      getBehaviorMessager().submitMessage(BuildingExplorationBehaviorAPI.IgnoreDebris, true);
   }

   @FXML
   public void confirmDoor()
   {
      getBehaviorMessager().submitMessage(BuildingExplorationBehaviorAPI.ConfirmDoor, true);
   }

   @FXML
   public void approveStairsSteps()
   {
      executeStairsStepsPublisher.publish(new Empty());
   }

   @FXML
   public void replanStairsSteps()
   {
      replanStairsStepsPublisher.publish(new Empty());
   }

   @Override
   public void destroy()
   {
      robotVisualizer.stop();
      lookAndStepVisualizationGroup.destroy();
      footstepPlanGraphic.destroy();
   }
}
