package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.QuadrupedPawPlannerNodeRejectionReason;

import java.util.concurrent.atomic.AtomicInteger;

public class FootstepPlannerVisualizationController
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
   private ComboBox<QuadrupedPawPlannerNodeRejectionReason> rejectionReasonToShow;
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
      ObservableList<QuadrupedPawPlannerNodeRejectionReason> plannerTypeOptions = FXCollections
            .observableArrayList(QuadrupedPawPlannerNodeRejectionReason.values);
      rejectionReasonToShow.setItems(plannerTypeOptions);
      rejectionReasonToShow.setValue(QuadrupedPawPlannerNodeRejectionReason.OBSTACLE_BLOCKING_STEP);
   }

   public void bindControls()
   {
      setupControls();

      messager.bindBidirectional(PawPlannerMessagerAPI.ShowAllValidNodesTopic, showAllValidNodes.selectedProperty(), false);
      messager.bindBidirectional(PawPlannerMessagerAPI.ShowAllInvalidNodesTopic, showAllInvalidNodes.selectedProperty(), false);
      messager.bindBidirectional(PawPlannerMessagerAPI.ShowNodesThisTickTopic, showNodesThisTick.selectedProperty(), false);
      messager.bindBidirectional(PawPlannerMessagerAPI.ShowNodesRejectedByReasonTopic, showNodesRejectedByReason.selectedProperty(), false);

      messager.bindBidirectional(PawPlannerMessagerAPI.RejectionReasonToShowTopic, rejectionReasonToShow.valueProperty(), false);

      messager.bindBidirectional(PawPlannerMessagerAPI.PlannerThoughtPlaybackFractionTopic, plannerPlaybackSlider.valueProperty(), false);

      messager.registerTopicListener(PawPlannerMessagerAPI.NodesThisTickTopic,
                                     nodes -> plannerPlaybackSlider.setBlockIncrement(1.0 / bufferSize.incrementAndGet()));
      messager.registerTopicListener(PawPlannerMessagerAPI.ComputePathTopic, data -> bufferSize.set(0));
   }
}
