package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import java.util.concurrent.atomic.AtomicInteger;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawStepPlannerNodeRejectionReason;

public class PawStepPlannerVisualizationController
{
   @FXML
   private CheckBox showAllValidNodes;
   @FXML
   private CheckBox showAllInvalidNodes;
   @FXML
   private CheckBox showNodesThisTick;
   @FXML
   private CheckBox showNodesRejectedByReason;
   @FXML
   private ComboBox<PawStepPlannerNodeRejectionReason> rejectionReasonToShow;
   @FXML
   private Slider plannerPlaybackSlider;
   //   @FXML
   //   public void requestStatistics()
   //   {
   //      throw new RuntimeException("This feature is currently not implemented.");
   //      if (verbose)
   //         PrintTools.info(this, "Clicked request statistics...");
   //
   //      messager.submitMessage(FootstepPlannerMessagerAPI.RequestPlannerStatistics, true);
   //   }

   private JavaFXMessager messager;

   private AtomicInteger bufferSize = new AtomicInteger(0);

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setupControls()
   {
      ObservableList<PawStepPlannerNodeRejectionReason> plannerTypeOptions = FXCollections
            .observableArrayList(PawStepPlannerNodeRejectionReason.values);
      rejectionReasonToShow.setItems(plannerTypeOptions);
      rejectionReasonToShow.setValue(PawStepPlannerNodeRejectionReason.OBSTACLE_BLOCKING_STEP);
   }

   public void bindControls()
   {
      setupControls();

      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowAllValidNodesTopic, showAllValidNodes.selectedProperty(), false);
      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowAllInvalidNodesTopic, showAllInvalidNodes.selectedProperty(), false);
      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowNodesThisTickTopic, showNodesThisTick.selectedProperty(), false);
      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowNodesRejectedByReasonTopic, showNodesRejectedByReason.selectedProperty(), false);

      messager.bindBidirectional(PawStepPlannerMessagerAPI.RejectionReasonToShowTopic, rejectionReasonToShow.valueProperty(), false);

      messager.bindBidirectional(PawStepPlannerMessagerAPI.PlannerThoughtPlaybackFractionTopic, plannerPlaybackSlider.valueProperty(), false);

      messager.addTopicListener(PawStepPlannerMessagerAPI.NodesThisTickTopic,
                                     nodes -> plannerPlaybackSlider.setBlockIncrement(1.0 / bufferSize.incrementAndGet()));
      messager.addTopicListener(PawStepPlannerMessagerAPI.ComputePathTopic, data -> bufferSize.set(0));
   }
}
