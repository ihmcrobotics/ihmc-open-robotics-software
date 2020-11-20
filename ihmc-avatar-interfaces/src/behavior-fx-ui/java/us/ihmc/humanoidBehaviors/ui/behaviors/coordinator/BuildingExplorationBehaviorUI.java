package us.ihmc.humanoidBehaviors.ui.behaviors.coordinator;

import controller_msgs.msg.dds.FootstepDataListMessage;
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
import us.ihmc.log.LogTools;
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

   private final LookAndStepVisualizationGroup lookAndStepVisualizationGroup;
   private final FootstepPlanGraphic stairsFootstepPlanGraphic;
   private final FootstepPlanGraphic controllerFootstepPlanGraphic;
   private final WalkingGoalPlacementEditor walkingGoalPlacementEditor = new WalkingGoalPlacementEditor();
   private final IHMCROS2Publisher<Empty> executeStairsStepsPublisher;
   private final IHMCROS2Publisher<Empty> replanStairsStepsPublisher;
   private final PositionGraphic goalGraphic;

   public BuildingExplorationBehaviorUI(SubScene subScene, Pane visualizationPane, ROS2NodeInterface ros2Node, Messager messager, DRCRobotModel robotModel)
   {
      super(subScene, visualizationPane, ros2Node, messager, robotModel);

      String robotName = robotModel.getSimpleRobotName();

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    controller_msgs.msg.dds.RobotConfigurationData.class,
                                                    ControllerAPIDefinition.getOutputTopic(robotName),
                                                    s -> getBehaviorMessager().submitMessage(BuildingExplorationBehaviorAPI.RobotConfigurationData,
                                                                                             s.takeNextData()));

      lookAndStepVisualizationGroup = new LookAndStepVisualizationGroup(ros2Node, messager);
      lookAndStepVisualizationGroup.setEnabled(true);
      stairsFootstepPlanGraphic = new FootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      stairsFootstepPlanGraphic.setTransparency(0.5);
      new IHMCROS2Callback<>(ros2Node, TraverseStairsBehaviorAPI.PLANNED_STEPS, footstepDataListMessage ->
            stairsFootstepPlanGraphic.generateMeshesAsynchronously(MinimalFootstep.convertFootstepDataListMessage(footstepDataListMessage)));

      controllerFootstepPlanGraphic = new FootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      new IHMCROS2Callback<>(ros2Node, ControllerAPIDefinition.getTopic(FootstepDataListMessage.class, robotName), footstepDataListMessage ->
            controllerFootstepPlanGraphic.generateMeshesAsynchronously(MinimalFootstep.convertFootstepDataListMessage(footstepDataListMessage)));

      goalGraphic = new PositionGraphic(Color.GRAY, 0.05);
      goalGraphic.setMouseTransparent(true);
      messager.registerTopicListener(Goal, newGoal -> goalGraphic.setPosition(newGoal.getPosition()));

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
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      enable3DGroup(enabled,
                    lookAndStepVisualizationGroup,
                    stairsFootstepPlanGraphic,
                    controllerFootstepPlanGraphic,
                    goalGraphic.getNode(),
                    walkingGoalPlacementEditor);
   }

   @FXML
   public void requestStart()
   {
      LogTools.info("Requesting start");
      getBehaviorMessager().submitMessage(BuildingExplorationBehaviorAPI.Start, true);
   }

   @FXML
   public void requestStop()
   {
      LogTools.info("Requesting stop");
      getBehaviorMessager().submitMessage(BuildingExplorationBehaviorAPI.Stop, true);
   }

   @FXML
   public void placeGoal()
   {
      LogTools.info("Placing goal");
      walkingGoalPlacementEditor.startGoalPlacement();
   }

   @FXML
   public void ignoreDebris()
   {
      LogTools.info("Ignore debris pressed");
      getBehaviorMessager().submitMessage(BuildingExplorationBehaviorAPI.IgnoreDebris, true);
   }

   @FXML
   public void confirmDoor()
   {
      LogTools.info("Confirm door pressed");
      getBehaviorMessager().submitMessage(BuildingExplorationBehaviorAPI.ConfirmDoor, true);
   }

   @FXML
   public void approveStairsSteps()
   {
      LogTools.info("Approve stairs pressed");
      executeStairsStepsPublisher.publish(new Empty());
   }

   @FXML
   public void replanStairsSteps()
   {
      LogTools.info("Replan stairs pressed");
      replanStairsStepsPublisher.publish(new Empty());
   }

   @Override
   public void destroy()
   {
      lookAndStepVisualizationGroup.destroy();
      stairsFootstepPlanGraphic.destroy();
   }
}
