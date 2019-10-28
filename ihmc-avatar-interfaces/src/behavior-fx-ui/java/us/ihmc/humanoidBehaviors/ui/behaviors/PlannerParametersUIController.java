package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.TunedFootstepPlannerParameters;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.messager.Messager;

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
   @FXML private Spinner<Double> stepUpHeight      ;
   @FXML private Spinner<Double> stepDownHeight    ;
   @FXML private Spinner<Double> maxStepUpX        ;
   @FXML private Spinner<Double> maxStepDownX      ;
   @FXML private Spinner<Double> timeout      ;
   @FXML private Spinner<Double> transferTimeFlatUp;
   @FXML private Spinner<Double> transferTimeDown  ;
   @FXML private Spinner<Double> swingTimeFlatUp   ;
   @FXML private Spinner<Double> swingTimeDown     ;

   private FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private Messager messager;

   public void init(Messager messager, DRCRobotModel robotModel)
   {
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      this.messager = messager;

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
      stepUpHeight        .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMaximumStepZWhenSteppingUp()      , 0.05   ));
      stepDownHeight      .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMaximumStepZWhenForwardAndDown()  , 0.05   ));
      maxStepUpX          .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMaximumStepReachWhenSteppingUp()  , 0.05   ));
      maxStepDownX        .setValueFactory(new DoubleSpinnerValueFactory(-10.0   ,10.0   ,footstepPlannerParameters.getMaximumStepXWhenForwardAndDown()  , 0.05   ));

      timeout.setValueFactory(new DoubleSpinnerValueFactory(0.0, 500.0, RemoteFootstepPlannerInterface.DEFAULT_TIMEOUT, 1.0));
      transferTimeFlatUp.setValueFactory(new DoubleSpinnerValueFactory(0.0, 500.0, RemoteFootstepPlannerInterface.DEFAULT_TRANSFER_TIME_FLAT_UP, 0.1));
      transferTimeDown  .setValueFactory(new DoubleSpinnerValueFactory(0.0, 500.0, RemoteFootstepPlannerInterface.DEFAULT_TRANSFER_TIME_DOWN   , 0.1));
      swingTimeFlatUp   .setValueFactory(new DoubleSpinnerValueFactory(0.0, 500.0, RemoteFootstepPlannerInterface.DEFAULT_SWING_TIME_FLAT_UP   , 0.1));
      swingTimeDown     .setValueFactory(new DoubleSpinnerValueFactory(0.0, 500.0, RemoteFootstepPlannerInterface.DEFAULT_SWING_TIME_DOWN      , 0.1));

      // add edit listeners to all fields and publish automatically
      cliffClearance.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      cliffHeight.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      maxStepLength.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      maxStepWidth.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      maxStepYaw.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      maxStepZ.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      maxXYWiggle.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      maxYawWiggle.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      minFootholdPercent.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      minStepLength.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      minStepWidth.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      minStepYaw.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      minSurfaceIncline.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      minXClearance.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      minYClearance.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      wiggleInsideDelta.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      stepUpHeight  .getValueFactory().valueProperty().addListener(observable -> publishParameters());
      stepDownHeight.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      maxStepUpX    .getValueFactory().valueProperty().addListener(observable -> publishParameters());
      maxStepDownX  .getValueFactory().valueProperty().addListener(observable -> publishParameters());
      timeout  .getValueFactory().valueProperty().addListener(observable -> publishParameters());
      transferTimeFlatUp.getValueFactory().valueProperty().addListener(observable -> publishParameters());
      transferTimeDown  .getValueFactory().valueProperty().addListener(observable -> publishParameters());
      swingTimeFlatUp   .getValueFactory().valueProperty().addListener(observable -> publishParameters());
      swingTimeDown     .getValueFactory().valueProperty().addListener(observable -> publishParameters());
      });
   }

   private void publishParameters()
   {
      TunedFootstepPlannerParameters tunedFootstepPlannerParameters = new TunedFootstepPlannerParameters();
      tunedFootstepPlannerParameters.setCliffClearance                   ( cliffClearance       .getValue());
      tunedFootstepPlannerParameters.setCliffHeight                      ( cliffHeight          .getValue());
      tunedFootstepPlannerParameters.setMaxStepLength                    ( maxStepLength        .getValue());
      tunedFootstepPlannerParameters.setMaxStepWidth                     ( maxStepWidth         .getValue());
      tunedFootstepPlannerParameters.setMaxStepYaw                       ( maxStepYaw           .getValue());
      tunedFootstepPlannerParameters.setMaxStepZ                         ( maxStepZ             .getValue());
      tunedFootstepPlannerParameters.setMaxXYWiggle                      ( maxXYWiggle          .getValue());
      tunedFootstepPlannerParameters.setMaxYawWiggle                     ( maxYawWiggle         .getValue());
      tunedFootstepPlannerParameters.setMinFootholdPercent               ( minFootholdPercent   .getValue());
      tunedFootstepPlannerParameters.setMinStepLength                    ( minStepLength        .getValue());
      tunedFootstepPlannerParameters.setMinStepWidth                     ( minStepWidth         .getValue());
      tunedFootstepPlannerParameters.setMinStepYaw                       ( minStepYaw           .getValue());
      tunedFootstepPlannerParameters.setMinSurfaceIncline                ( minSurfaceIncline    .getValue());
      tunedFootstepPlannerParameters.setMinXClearance                    ( minXClearance        .getValue());
      tunedFootstepPlannerParameters.setMinYClearance                    ( minYClearance        .getValue());
      tunedFootstepPlannerParameters.setWiggleInsideDelta                ( wiggleInsideDelta    .getValue());
      tunedFootstepPlannerParameters.setStepUpHeight                     ( stepUpHeight         .getValue());
      tunedFootstepPlannerParameters.setStepDownHeight                   ( stepDownHeight       .getValue());
      tunedFootstepPlannerParameters.setMaxStepUpX                       ( maxStepUpX           .getValue());
      tunedFootstepPlannerParameters.setMaxStepDownX                     ( maxStepDownX         .getValue());
      tunedFootstepPlannerParameters.setTimeout                          ( timeout              .getValue());
      tunedFootstepPlannerParameters.setTransferTimeFlatUp               ( transferTimeFlatUp   .getValue());
      tunedFootstepPlannerParameters.setTransferTimeDown                 ( transferTimeDown     .getValue());
      tunedFootstepPlannerParameters.setSwingTimeFlatUp                  ( swingTimeFlatUp      .getValue());
      tunedFootstepPlannerParameters.setSwingTimeDown                    ( swingTimeDown        .getValue());

      messager.submitMessage(PatrolBehaviorAPI.PlannerParameters, tunedFootstepPlannerParameters);
   }

   @FXML
   public void cancelPlanning()
   {
      messager.submitMessage(PatrolBehaviorAPI.CancelPlanning, new Object());
   }
}
