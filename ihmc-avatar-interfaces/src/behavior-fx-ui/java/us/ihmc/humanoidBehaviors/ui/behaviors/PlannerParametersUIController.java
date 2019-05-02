package us.ihmc.humanoidBehaviors.ui.behaviors;

import controller_msgs.msg.dds.ToolboxStateMessage;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.API;
import us.ihmc.humanoidBehaviors.tools.TunedFootstepPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2Node;

public class PlannerParametersUIController
{
   @FXML private Spinner<Double> cliffClearance;
   @FXML private Spinner<Double> cliffHeight;
   @FXML private Spinner<Double> maxStepLength;
   @FXML private Spinner<Double> maxStepWidth;
   @FXML private Spinner<Double> maxStepYaw;
   @FXML private Spinner<Double> maxStepZ;
   @FXML private Spinner<Double> maxXYWiggle;
   @FXML private Spinner<Double> maxYawWiggle;
   @FXML private Spinner<Double> minFootholdPercent;
   @FXML private Spinner<Double> minStepLength;
   @FXML private Spinner<Double> minStepWidth;
   @FXML private Spinner<Double> minStepYaw;
   @FXML private Spinner<Double> minSurfaceIncline;
   @FXML private Spinner<Double> minXClearance;
   @FXML private Spinner<Double> minYClearance;
   @FXML private Spinner<Double> wiggleInsideDelta;
   @FXML private Button cancelPlanning;

   private FootstepPlannerParameters footstepPlannerParameters;
   private Messager messager;
   private IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;

   public void init(Ros2Node ros2Node, DRCRobotModel robotModel, Messager messager)
   {
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      this.messager = messager;

      toolboxStatePublisher =
            ROS2Tools.createPublisher(ros2Node,
                                      ToolboxStateMessage.class,
                                      FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotModel.getSimpleRobotName()));

      Platform.runLater(() ->
      {
      cliffClearance      .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMinimumDistanceFromCliffBottoms() , 0.05   ));
      cliffHeight         .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getCliffHeightToAvoid()              , 0.05   ));
      maxStepLength       .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMaximumStepReach()                , 0.05   ));
      maxStepWidth        .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMaximumStepWidth()                , 0.05   ));
      maxStepYaw          .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMaximumStepYaw()                  , 0.05   ));
      maxStepZ            .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMaximumStepZ()                    , 0.05   ));
      maxXYWiggle         .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMaximumXYWiggleDistance()         , 0.05   ));
      maxYawWiggle        .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMaximumYawWiggle()                , 0.05   ));
      minFootholdPercent  .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMinimumFootholdPercent()          , 0.05   ));
      minStepLength       .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMinimumStepLength()               , 0.05   ));
      minStepWidth        .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMinimumStepWidth()                , 0.05   ));
      minStepYaw          .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMinimumStepYaw()                  , 0.05   ));
      minSurfaceIncline   .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMinimumSurfaceInclineRadians()    , 0.05   ));
      minXClearance       .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMinXClearanceFromStance()         , 0.05   ));
      minYClearance       .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMinYClearanceFromStance()         , 0.05   ));
      wiggleInsideDelta   .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getWiggleInsideDelta()               , 0.05   ));

      // add edit listeners to all fields and publish automatically
      cliffClearance.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      cliffHeight.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      maxStepLength.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      maxStepWidth.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      maxStepYaw.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      maxStepZ.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      maxXYWiggle.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      maxYawWiggle.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      minFootholdPercent.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      minStepLength.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      minStepWidth.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      minStepYaw.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      minSurfaceIncline.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      minXClearance.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      minYClearance.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      wiggleInsideDelta.getValueFactory().valueProperty().addListener((ChangeListener) -> publishParameters());
      });
   }

   private void publishParameters()
   {
      TunedFootstepPlannerParameters tunedFootstepPlannerParameters = new TunedFootstepPlannerParameters();
      tunedFootstepPlannerParameters.setCliffClearance      ( cliffClearance       .getValue()       );
      tunedFootstepPlannerParameters.setCliffHeight         ( cliffHeight          .getValue()       );
      tunedFootstepPlannerParameters.setMaxStepLength       ( maxStepLength        .getValue()       );
      tunedFootstepPlannerParameters.setMaxStepWidth        ( maxStepWidth         .getValue()       );
      tunedFootstepPlannerParameters.setMaxStepYaw          ( maxStepYaw           .getValue()       );
      tunedFootstepPlannerParameters.setMaxStepZ            ( maxStepZ             .getValue()       );
      tunedFootstepPlannerParameters.setMaxXYWiggle         ( maxXYWiggle          .getValue()       );
      tunedFootstepPlannerParameters.setMaxYawWiggle        ( maxYawWiggle         .getValue()       );
      tunedFootstepPlannerParameters.setMinFootholdPercent  ( minFootholdPercent   .getValue()       );
      tunedFootstepPlannerParameters.setMinStepLength       ( minStepLength        .getValue()       );
      tunedFootstepPlannerParameters.setMinStepWidth        ( minStepWidth         .getValue()       );
      tunedFootstepPlannerParameters.setMinStepYaw          ( minStepYaw           .getValue()       );
      tunedFootstepPlannerParameters.setMinSurfaceIncline   ( minSurfaceIncline    .getValue()       );
      tunedFootstepPlannerParameters.setMinXClearance       ( minXClearance        .getValue()       );
      tunedFootstepPlannerParameters.setMinYClearance       ( minYClearance        .getValue()       );
      tunedFootstepPlannerParameters.setWiggleInsideDelta   ( wiggleInsideDelta    .getValue()       );

      messager.submitMessage(API.PlannerParameters, tunedFootstepPlannerParameters);
   }

   @FXML
   public void cancelPlanning()
   {
      LogTools.debug("Cancel planning clicked. Sending SLEEP to footstep planner");
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }
}
