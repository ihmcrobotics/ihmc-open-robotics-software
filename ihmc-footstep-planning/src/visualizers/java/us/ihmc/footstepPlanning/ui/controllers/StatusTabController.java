package us.ihmc.footstepPlanning.ui.controllers;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.AbortPlanning;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ComputePath;

import java.util.concurrent.atomic.AtomicReference;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import us.ihmc.commons.PrintTools;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.TopicListener;

public class StatusTabController
{
   private static final boolean verbose = false;

   @FXML
   private ComboBox<FootstepPlannerType> plannerTypeComboBox;
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
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestId, newRequestID);
      messager.submitMessage(ComputePath, true);
   }
   
   @FXML
   public void abortPlanning()
   {
      if (verbose)
         PrintTools.info(this, "Clicked abort planning...");
      
      messager.submitMessage(AbortPlanning, true);;
   }

   private JavaFXMessager messager;


   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
      currentPlannerRequestId = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestId, -1);
   }

   private void setupControls()
   {
      ObservableList<FootstepPlannerType> plannerTypeOptions = FXCollections.observableArrayList(FootstepPlannerType.values);
      plannerTypeComboBox.setItems(plannerTypeOptions);
      plannerTypeComboBox.setValue(FootstepPlannerType.A_STAR);
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

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerType, plannerTypeComboBox.valueProperty(), true);
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.PlannerRequestId, new TextViewerListener<>(requestID));
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.ReceivedPlanId, new TextViewerListener<>(receivedRequestId));
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.PlannerTimeTaken, new TextViewerListener<>(timeTaken));
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.PlanningResult, new TextViewerListener<>(planningResult));
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.PlannerStatus, new TextViewerListener<>(plannerStatus));

      messager.bindBidirectional(FootstepPlannerMessagerAPI.AcceptNewPlanarRegions, acceptNewPlanarRegions.selectedProperty(), true);

   }


}
