package us.ihmc.humanoidBehaviors.ui.behaviors.coordinator;

import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.text.Text;
import javafx.stage.Stage;
import std_msgs.msg.dds.Empty;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationBehaviorAPI;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationStateName;
import us.ihmc.humanoidBehaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepVisualizationGroup;
import us.ihmc.humanoidBehaviors.ui.editors.WalkingGoalPlacementEditor;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;
import us.ihmc.ros2.ROS2Node;

import static us.ihmc.humanoidBehaviors.demo.BuildingExplorationBehaviorAPI.*;

public class BuildingExplorationBehaviorUI extends Group
{
   @FXML private ComboBox<BuildingExplorationStateName> requestedState;
   @FXML private Spinner<Double> goalX;
   @FXML private Spinner<Double> goalY;
   @FXML private Spinner<Double> goalZ;
   @FXML private Text currentState;
   @FXML private Text debrisDetected;
   @FXML private Text stairsDetected;
   @FXML private Text doorDetected;
   @FXML private Button placeGoal;

   private final Stage primaryStage;
   private final JavaFXRobotVisualizer robotVisualizer;
   private final LookAndStepVisualizationGroup lookAndStepVisualizationGroup;
   private final FootstepPlanGraphic footstepPlanGraphic;
   private final ROS2Node ros2Node;
   private final Messager messager;
   private final Point3DProperty goalProperty = new Point3DProperty(this, "goalProperty", new Point3D());
   private final WalkingGoalPlacementEditor walkingGoalPlacementEditor = new WalkingGoalPlacementEditor();
   private final IHMCROS2Publisher<Empty> executeStairsStepsPublisher;
   private final IHMCROS2Publisher<Empty> replanStairsStepsPublisher;

   public BuildingExplorationBehaviorUI(Stage primaryStage,
                                        JavaFXMessager messager,
                                        DRCRobotModel robotModel,
                                        ROS2Node ros2Node,
                                        Messager behaviorMessager) throws Exception
   {
      this.primaryStage = primaryStage;
      this.ros2Node = ros2Node;
      this.messager = messager;
      primaryStage.setTitle(getClass().getSimpleName());

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    controller_msgs.msg.dds.RobotConfigurationData.class,
                                                    ControllerAPIDefinition.getOutputTopic(robotModel.getSimpleRobotName()),
                                                    s -> messager.submitMessage(BuildingExplorationBehaviorAPI.RobotConfigurationData, s.takeNextData()));

      BorderPane mainPane = new BorderPane();
      GridPane thisPane = JavaFXMissingTools.loadFromFXML(this);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addDefaultLighting();

      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      worldCoordinateSystem.setMouseTransparent(true);
      view3dFactory.addNodeToView(worldCoordinateSystem);

      robotVisualizer = new JavaFXRobotVisualizer(robotModel);
      lookAndStepVisualizationGroup = new LookAndStepVisualizationGroup(ros2Node, behaviorMessager);
      lookAndStepVisualizationGroup.setEnabled(true);
      footstepPlanGraphic = new FootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      new IHMCROS2Callback<>(ros2Node, TraverseStairsBehaviorAPI.PLANNED_STEPS, footstepDataListMessage ->
            footstepPlanGraphic.generateMeshesAsynchronously(MinimalFootstep.convertFootstepDataListMessage(footstepDataListMessage)));

      robotVisualizer.start();

      PositionGraphic goalGraphic = new PositionGraphic(Color.GRAY, 0.05);
      view3dFactory.addNodeToView(goalGraphic.getNode());
      goalGraphic.setMouseTransparent(true);
      messager.registerTopicListener(Goal, newGoal -> goalGraphic.setPosition(newGoal.getPosition()));

      messager.registerTopicListener(RobotConfigurationData, robotVisualizer::submitNewConfiguration);
      view3dFactory.addNodeToView(robotVisualizer.getRootNode());
      view3dFactory.addNodeToView(lookAndStepVisualizationGroup);
      view3dFactory.addNodeToView(footstepPlanGraphic);

      requestedState.setItems(FXCollections.observableArrayList(BuildingExplorationStateName.values()));

      goalX.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      goalY.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      goalZ.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));

      //      messager.bindBidirectional(Goal, goalProperty, false);
      goalProperty.bindBidirectionalX(goalX.getValueFactory().valueProperty());
      goalProperty.bindBidirectionalY(goalY.getValueFactory().valueProperty());
      goalProperty.bindBidirectionalZ(goalZ.getValueFactory().valueProperty());

      goalX.getValueFactory().setValue(14.2);
      goalZ.getValueFactory().setValue(0.86);

      currentState.setText("----");
      debrisDetected.setText("No");
      stairsDetected.setText("No");
      doorDetected.setText("No");

      this.messager.registerTopicListener(CurrentState, state ->
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

      this.messager.registerTopicListener(DebrisDetected, d -> debrisDetected.setText("Yes"));
      this.messager.registerTopicListener(StairsDetected, d -> stairsDetected.setText("Yes"));
      this.messager.registerTopicListener(DoorDetected, d -> doorDetected.setText("Yes"));

      requestedState.getSelectionModel()
                    .selectedItemProperty()
                    .addListener((observable, oldState, newState) -> this.messager.submitMessage(BuildingExplorationBehaviorAPI.RequestedState, newState));

      executeStairsStepsPublisher = new IHMCROS2Publisher<>(this.ros2Node, TraverseStairsBehaviorAPI.EXECUTE_STEPS);
      replanStairsStepsPublisher = new IHMCROS2Publisher<>(this.ros2Node, TraverseStairsBehaviorAPI.REPLAN);

      walkingGoalPlacementEditor.init(view3dFactory.getSubScene(), placeGoal, placedGoal -> this.messager.submitMessage(BuildingExplorationBehaviorAPI.Goal, placedGoal));
      getChildren().add(walkingGoalPlacementEditor);

      mainPane.setBottom(thisPane);

      view3dFactory.addNodeToView(this);

      mainPane.setCenter(subScene);

      Scene mainScene = new Scene(mainPane);
      primaryStage.setScene(mainScene);
      primaryStage.setWidth(1400);
      primaryStage.setHeight(950);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   @FXML
   public void requestStart()
   {
      messager.submitMessage(BuildingExplorationBehaviorAPI.Start, true);
   }

   @FXML
   public void requestStop()
   {
      messager.submitMessage(BuildingExplorationBehaviorAPI.Stop, true);
   }

   @FXML
   public void placeGoal()
   {
      walkingGoalPlacementEditor.startGoalPlacement();
   }

   @FXML
   public void ignoreDebris()
   {
      messager.submitMessage(BuildingExplorationBehaviorAPI.IgnoreDebris, true);
   }

   @FXML
   public void confirmDoor()
   {
      messager.submitMessage(BuildingExplorationBehaviorAPI.ConfirmDoor, true);
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

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {
      robotVisualizer.stop();
      lookAndStepVisualizationGroup.destroy();
      ros2Node.destroy();
      footstepPlanGraphic.destroy();
   }
}
