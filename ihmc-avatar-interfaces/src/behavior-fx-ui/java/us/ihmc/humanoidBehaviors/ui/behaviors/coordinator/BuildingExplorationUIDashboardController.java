package us.ihmc.humanoidBehaviors.ui.behaviors.coordinator;

import controller_msgs.msg.dds.FootstepDataListMessage;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.text.Text;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationStateName;
import us.ihmc.humanoidBehaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;
import us.ihmc.ros2.ROS2Node;

import static us.ihmc.humanoidBehaviors.ui.behaviors.coordinator.BuildingExplorationBehaviorAPI.*;

public class BuildingExplorationUIDashboardController
{
   @FXML
   private ComboBox<BuildingExplorationStateName> requestedState;

   @FXML
   private Spinner<Double> goalX;
   @FXML
   private Spinner<Double> goalY;
   @FXML
   private Spinner<Double> goalZ;

   @FXML
   private Text currentState;
   @FXML
   private Text debrisDetected;
   @FXML
   private Text stairsDetected;
   @FXML
   private Text doorDetected;

   private Messager messager;
   private final Point3DProperty goalProperty = new Point3DProperty(this, "goalProperty", new Point3D());

   private IHMCROS2Publisher<Empty> executeStairsStepsPublisher;
   private IHMCROS2Publisher<Empty> replanStairsStepsPublisher;

   public void bindControls(JavaFXMessager messager, ROS2Node ros2Node)
   {
      this.messager = messager;
      requestedState.setItems(FXCollections.observableArrayList(BuildingExplorationStateName.values()));

      goalX.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      goalY.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      goalZ.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));

      messager.bindBidirectional(Goal, goalProperty, false);
      goalProperty.bindBidirectionalX(goalX.getValueFactory().valueProperty());
      goalProperty.bindBidirectionalY(goalY.getValueFactory().valueProperty());
      goalProperty.bindBidirectionalZ(goalZ.getValueFactory().valueProperty());

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
      messager.submitMessage(BuildingExplorationBehaviorAPI.PlaceGoal, true);
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
}
