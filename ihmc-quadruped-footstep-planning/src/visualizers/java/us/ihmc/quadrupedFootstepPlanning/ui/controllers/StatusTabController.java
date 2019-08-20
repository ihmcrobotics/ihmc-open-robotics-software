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
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlannerType;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawPlannerMessagerAPI;

import java.util.concurrent.atomic.AtomicReference;

public class StatusTabController
{
   private static final boolean verbose = false;

   @FXML
   private ComboBox<PawPlannerType> plannerTypeComboBox;
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
      messager.submitMessage(PawPlannerMessagerAPI.PlannerRequestIdTopic, newRequestID);
      messager.submitMessage(PawPlannerMessagerAPI.ComputePathTopic, true);
   }

   @FXML
   public void abortPlanning()
   {
      if (verbose)
         PrintTools.info(this, "Clicked abort planning...");

      messager.submitMessage(PawPlannerMessagerAPI.AbortPlanningTopic, true);
   }

   private JavaFXMessager messager;


   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
      currentPlannerRequestId = messager.createInput(PawPlannerMessagerAPI.PlannerRequestIdTopic, -1);
   }

   private void setupControls()
   {
      ObservableList<PawPlannerType> plannerTypeOptions = FXCollections.observableArrayList(PawPlannerType.values);
      plannerTypeComboBox.setItems(plannerTypeOptions);
      plannerTypeComboBox.setValue(PawPlannerType.A_STAR);
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

      messager.bindBidirectional(PawPlannerMessagerAPI.PlannerTypeTopic, plannerTypeComboBox.valueProperty(), true);
      messager.registerJavaFXSyncedTopicListener(PawPlannerMessagerAPI.PlannerRequestIdTopic, new TextViewerListener<>(requestID));
      messager.registerJavaFXSyncedTopicListener(PawPlannerMessagerAPI.ReceivedPlanIdTopic, new TextViewerListener<>(receivedRequestId));
      messager.registerJavaFXSyncedTopicListener(PawPlannerMessagerAPI.PlannerTimeTakenTopic, new TextViewerListener<>(timeTaken));
      messager.registerJavaFXSyncedTopicListener(PawPlannerMessagerAPI.PlanningResultTopic, new TextViewerListener<>(planningResult));
      messager.registerJavaFXSyncedTopicListener(PawPlannerMessagerAPI.PlannerStatusTopic, new TextViewerListener<>(plannerStatus));

      messager.bindBidirectional(PawPlannerMessagerAPI.AcceptNewPlanarRegionsTopic, acceptNewPlanarRegions.selectedProperty(), true);

   }


}
