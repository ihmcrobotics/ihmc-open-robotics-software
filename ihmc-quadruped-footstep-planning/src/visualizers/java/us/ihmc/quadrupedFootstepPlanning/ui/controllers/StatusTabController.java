package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.TopicListener;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;

import java.util.concurrent.atomic.AtomicReference;

public class StatusTabController
{
   private static final boolean verbose = false;

   @FXML
   private ComboBox<PawStepPlannerType> plannerTypeComboBox;
   @FXML
   private TextField requestID;
   @FXML
   private TextField receivedRequestId;
   @FXML
   private TextField timeTaken;
   @FXML
   private TextField plannerStatus;
   @FXML
   private TextField planningResult;
   @FXML
   private ToggleButton acceptNewPlanarRegions;

   private AtomicReference<Integer> currentPlannerRequestId;

   @FXML
   public void computePath()
   {
      if (verbose)
         PrintTools.info(this, "Clicked compute path...");

      int newRequestID = currentPlannerRequestId.get() + 1;
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerRequestIdTopic, newRequestID);
      messager.submitMessage(PawStepPlannerMessagerAPI.ComputePathTopic, true);
   }

   @FXML
   public void abortPlanning()
   {
      if (verbose)
         PrintTools.info(this, "Clicked abort planning...");

      messager.submitMessage(PawStepPlannerMessagerAPI.AbortPlanningTopic, true);
   }

   private JavaFXMessager messager;


   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
      currentPlannerRequestId = messager.createInput(PawStepPlannerMessagerAPI.PlannerRequestIdTopic, -1);
   }

   private void setupControls()
   {
      ObservableList<PawStepPlannerType> plannerTypeOptions = FXCollections.observableArrayList(PawStepPlannerType.values);
      plannerTypeComboBox.setItems(plannerTypeOptions);
      plannerTypeComboBox.setValue(PawStepPlannerType.A_STAR);
   }

   private class TextViewerListener<T> implements TopicListener<T>
   {
      private final TextField textField;
      public TextViewerListener(TextField textField)
      {
         this.textField = textField;
      }

      public void receivedMessageForTopic(T messageContent)
      {
         if (messageContent != null)
            textField.promptTextProperty().setValue(messageContent.toString());
      }
   }

   public void bindControls()
   {
      setupControls();

      messager.bindBidirectional(PawStepPlannerMessagerAPI.PlannerTypeTopic, plannerTypeComboBox.valueProperty(), true);
      messager.registerJavaFXSyncedTopicListener(PawStepPlannerMessagerAPI.PlannerRequestIdTopic, new TextViewerListener<>(requestID));
      messager.registerJavaFXSyncedTopicListener(PawStepPlannerMessagerAPI.ReceivedPlanIdTopic, new TextViewerListener<>(receivedRequestId));
      messager.registerJavaFXSyncedTopicListener(PawStepPlannerMessagerAPI.PlannerTimeTakenTopic, new TextViewerListener<>(timeTaken));
      messager.registerJavaFXSyncedTopicListener(PawStepPlannerMessagerAPI.PlanningResultTopic, new TextViewerListener<>(planningResult));
      messager.registerJavaFXSyncedTopicListener(PawStepPlannerMessagerAPI.PlannerStatusTopic, new TextViewerListener<>(plannerStatus));

      messager.bindBidirectional(PawStepPlannerMessagerAPI.AcceptNewPlanarRegionsTopic, acceptNewPlanarRegions.selectedProperty(), true);

   }


}
