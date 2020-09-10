package us.ihmc.humanoidBehaviors.ui.behaviors.coordinator;

import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.text.Text;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.demo.BuildingExplorationStateName;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;

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

   private Messager messager;
   private final Point3DProperty goalProperty = new Point3DProperty(this, "goalProperty", new Point3D());

   public void bindControls(JavaFXMessager messager)
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

      messager.registerTopicListener(CurrentState, state ->
      {
         currentState.setText(state.toString());
         if (state == BuildingExplorationStateName.LOOK_AND_STEP)
         {
            debrisDetected.setText("No");
            stairsDetected.setText("No");
         }
      });

      messager.registerTopicListener(DebrisDetected, d -> debrisDetected.setText("Yes"));
      messager.registerTopicListener(StairsDetected, d -> stairsDetected.setText("Yes"));
   }

   @FXML
   public void requestStart()
   {
      messager.submitMessage(BuildingExplorationBehaviorAPI.RequestedState, requestedState.getValue());
      messager.submitMessage(BuildingExplorationBehaviorAPI.Start, true);
   }

   @FXML
   public void requestStop()
   {
      messager.submitMessage(BuildingExplorationBehaviorAPI.Stop, true);
   }
}
