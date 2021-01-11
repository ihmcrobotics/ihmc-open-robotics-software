package us.ihmc.footstepPlanning.ui.controllers;

import controller_msgs.msg.dds.*;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.HBox;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.ui.UIAuxiliaryRobotData;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;

public class RobotOperationTabController
{
   private JavaFXMessager messager;
   private RobotLowLevelMessenger robotLowLevelMessenger;

   @FXML private Button homeAll;
   @FXML private Button freeze;
   @FXML private Button standPrep;
   @FXML private Button shutdown;
   @FXML private Button abortWalking;
   @FXML private Button pauseWalking;
   @FXML private Button continueWalking;
   @FXML private CheckBox enableSupportRegions;
   @FXML private Spinner<Double> supportRegionScale;
   @FXML private HBox robotOperationHBox;

   private IHMCRealtimeROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;

   private final RobotIKUI robotIKUI = new RobotIKUI();

   public RobotOperationTabController()
   {
   }

   /** Called by JavaFX via reflection */
   public void initialize()
   {
      AnchorPane robotIKUIPane = JavaFXMissingTools.loadFromFXML(robotIKUI);
      robotOperationHBox.getChildren().add(robotIKUIPane);
      updateButtons();
      supportRegionScale.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 2.0, 0.1));
   }

   private void updateButtons()
   {
      homeAll.setDisable(messager == null);
      freeze.setDisable(robotLowLevelMessenger == null);
      standPrep.setDisable(robotLowLevelMessenger == null);
      shutdown.setDisable(robotLowLevelMessenger == null);
      abortWalking.setDisable(robotLowLevelMessenger == null);
      pauseWalking.setDisable(robotLowLevelMessenger == null);
   }

   @FXML
   public void homeAll()
   {
      GoHomeMessage homeLeftArm = new GoHomeMessage();
      homeLeftArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeLeftArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_LEFT);
      messager.submitMessage(FootstepPlannerMessagerAPI.GoHomeTopic, homeLeftArm);

      GoHomeMessage homeRightArm = new GoHomeMessage();
      homeRightArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeRightArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_RIGHT);
      messager.submitMessage(FootstepPlannerMessagerAPI.GoHomeTopic, homeRightArm);
   }

   @FXML
   public void freeze()
   {
      robotLowLevelMessenger.sendFreezeRequest();
   }

   @FXML
   public void standPrep()
   {
      robotLowLevelMessenger.sendStandRequest();
   }

   @FXML
   public void shutdown()
   {
      robotLowLevelMessenger.sendShutdownRequest();
   }

   @FXML
   public void abortWalking()
   {
      robotLowLevelMessenger.sendAbortWalkingRequest();
   }

   @FXML
   public void pauseWalking()
   {
      robotLowLevelMessenger.sendPauseWalkingRequest();
   }

   @FXML
   public void continueWalking()
   {
      robotLowLevelMessenger.sendContinueWalkingRequest();
   }

   @FXML
   public void sendSupportRegionParameters()
   {
      BipedalSupportPlanarRegionParametersMessage supportPlanarRegionParametersMessage = new BipedalSupportPlanarRegionParametersMessage();
      supportPlanarRegionParametersMessage.setEnable(enableSupportRegions.isSelected());
      supportPlanarRegionParametersMessage.setSupportRegionScaleFactor(supportRegionScale.getValue());
      messager.submitMessage(FootstepPlannerMessagerAPI.BipedalSupportRegionsParameters, supportPlanarRegionParametersMessage);
   }

   @FXML
   public void clearREA()
   {
      REAStateRequestMessage clearMessage = new REAStateRequestMessage();
      clearMessage.setRequestClear(true);
      reaStateRequestPublisher.publish(clearMessage);
   }

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
      updateButtons();
   }

   public void setRobotLowLevelMessenger(RobotLowLevelMessenger robotLowLevelMessenger)
   {
      this.robotLowLevelMessenger = robotLowLevelMessenger;
      updateButtons();
   }

   public void setREAStateRequestPublisher(IHMCRealtimeROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher)
   {
      this.reaStateRequestPublisher = reaStateRequestPublisher;
   }

   public void setAuxiliaryRobotData(UIAuxiliaryRobotData auxiliaryRobotData)
   {
      robotIKUI.setAuxiliaryRobotData(auxiliaryRobotData);
   }

   public void setFullRobotModel(FullHumanoidRobotModel fullRobotModel, FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory)
   {
      robotIKUI.setFullRobotModel(fullRobotModel, fullHumanoidRobotModelFactory);
   }
}
