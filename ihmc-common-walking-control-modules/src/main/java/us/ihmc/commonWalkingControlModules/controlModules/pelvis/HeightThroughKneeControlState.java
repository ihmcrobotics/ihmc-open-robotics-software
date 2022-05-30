package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.TaskspaceTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.*;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.heightPlanning.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.humanoidRobotics.model.CenterOfMassStateProvider;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class HeightThroughKneeControlState implements PelvisAndCenterOfMassHeightControlState
{

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean controlPelvisHeightInsteadOfCoMHeight = new YoBoolean("controlPelvisHeightInsteadOfCoMHeight", registry);
   private final YoBoolean controlHeightWithMomentum = new YoBoolean("controlHeightWithMomentum", registry);


   private final YoDouble hackKp = new YoDouble("hackKp", registry);
   private final YoDouble hackDesiredKneeAngle = new YoDouble("hackDesiredKneeAngle", registry);
   private final YoDouble hackStraightestKneeAngle = new YoDouble("hackStraightestKneeAngle", registry);
   private final YoDouble hackZDesired = new YoDouble("hackZDesired", registry);
   private final YoDouble hackZCurrent = new YoDouble("hackZCurrent", registry);
   private final YoEnum<RobotSide> kneeSideToControl = new YoEnum<>("kneeSideToControl", registry, RobotSide.class);
   private final YoEnum<RobotSide> supportLegSide = new YoEnum<>("kneeControlSupportLegSide", registry, RobotSide.class);


   private final DoubleProvider currentTime;
   private final YoDouble transitionDurationToFall = new YoDouble("comHeightTransitionDurationToFall", registry);
   private final YoDouble transitionToFallStartTime = new YoDouble("comHeightTransitionToFallStartTime", registry);
   private final YoDouble fallActivationRatio = new YoDouble("comHeightFallActivationRatio", registry);

   private final ReferenceFrame centerOfMassFrame;
   private final MovingReferenceFrame pelvisFrame;

   private final PointFeedbackControlCommand pelvisHeightControlCommand = new PointFeedbackControlCommand();
   private final SideDependentList<OneDoFJointFeedbackControlCommand> kneeControlCommands;
   private final YoPDGains kneeGains = new YoPDGains("kneeGains", registry);

   private final FullHumanoidRobotModel fullRobotModel;

   private final SideDependentList<OneDoFJointBasics> kneeJoints;
   private final YoDouble leftDesiredSupportKneeAngle = new YoDouble("leftDesiredSupportKneeAngle", registry);
   private final YoDouble rightDesiredSupportKneeAngle = new YoDouble("rightDesiredSupportKneeAngle", registry);
   private final SideDependentList<YoDouble> desiredSupportKneeAngles = new SideDependentList<>(leftDesiredSupportKneeAngle, rightDesiredSupportKneeAngle);

   public HeightThroughKneeControlState(HighLevelHumanoidControllerToolbox controllerToolbox,
                                        WalkingControllerParameters walkingControllerParameters,
                                        YoRegistry parentRegistry)
   {
      leftDesiredSupportKneeAngle.set(0.5);
      rightDesiredSupportKneeAngle.set(0.5);

      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      fullRobotModel = controllerToolbox.getFullRobotModel();

      OneDoFJointBasics leftKneePitch = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH);
      OneDoFJointBasics rightKneePitch = fullRobotModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH);
      kneeJoints = new SideDependentList<>(leftKneePitch, rightKneePitch);

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      pelvisFrame = referenceFrames.getPelvisFrame();

      // TODO: Fix low level stuff so that we are truly controlling pelvis height and not CoM height.
      controlPelvisHeightInsteadOfCoMHeight.set(walkingControllerParameters.controlPelvisHeightInsteadOfCoMHeight());
      controlHeightWithMomentum.set(walkingControllerParameters.controlHeightWithMomentum());

      currentTime = controllerToolbox.getYoTime();

      SelectionMatrix3D selectionMatrix = new SelectionMatrix3D(worldFrame, false, false, true);
      pelvisHeightControlCommand.set(fullRobotModel.getElevator(), fullRobotModel.getPelvis());
      FramePoint3D pelvisPoint = new FramePoint3D(pelvisFrame);
      pelvisPoint.changeFrame(fullRobotModel.getPelvis().getBodyFixedFrame());
      pelvisHeightControlCommand.setBodyFixedPointToControl(pelvisPoint);
      pelvisHeightControlCommand.setSelectionMatrix(selectionMatrix);

      OneDoFJointFeedbackControlCommand leftKneeControlCommand = new OneDoFJointFeedbackControlCommand();
      OneDoFJointFeedbackControlCommand rightKneeControlCommand = new OneDoFJointFeedbackControlCommand();

      leftKneeControlCommand.setJoint(leftKneePitch);
      rightKneeControlCommand.setJoint(rightKneePitch);

      leftKneeControlCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      rightKneeControlCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

      leftKneeControlCommand.setWeightForSolver(10.0);
      rightKneeControlCommand.setWeightForSolver(10.0);

      kneeGains.setKp(100.0);
      kneeGains.setKd(GainCalculator.computeDerivativeGain(100.0, 0.7));
      //      kneeGains.setZeta(0.7);

      leftKneeControlCommand.setGains(kneeGains);
      rightKneeControlCommand.setGains(kneeGains);

      kneeControlCommands = new SideDependentList<>(leftKneeControlCommand, rightKneeControlCommand);

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      transitionToFallStartTime.setToNaN();
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
      transitionToFallStartTime.setToNaN();
   }

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      if (!transitionToFallStartTime.isNaN())
         initialize();
   }

   public void initializeTransitionToFall(double transitionDuration)
   {
      if (transitionToFallStartTime.isNaN())
      {
         transitionToFallStartTime.set(currentTime.getValue());
         transitionDurationToFall.set(transitionDuration);
      }
   }

   @Override
   public void goHome(double trajectoryTime)
   {
   }

   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      supportLegSide.set(supportLeg);
   }



   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D desiredAcceleration = new FrameVector3D();
   private PDGainsReadOnly gains;
   // Temporary objects to reduce garbage collection.
   private final FramePoint3D comPosition = new FramePoint3D();

   private final FramePoint3D pelvisPosition = new FramePoint3D();

   @Override
   public void computeCoMHeightCommand(FrameVector2DReadOnly desiredICPVelocity,
                                       FrameVector2DReadOnly desiredCoMVelocity,
                                       boolean isInDoubleSupport,
                                       double omega0,
                                       FeetManager feetManager)
   {
      comPosition.setToZero(centerOfMassFrame);
      comPosition.changeFrame(worldFrame);

      double zCurrent = comPosition.getZ();

      // FIXME should we be using the desired values instead of the current values?
      if (controlPelvisHeightInsteadOfCoMHeight.getBooleanValue())
      {
         pelvisPosition.setToZero(pelvisFrame);
         pelvisPosition.changeFrame(worldFrame);
         zCurrent = pelvisPosition.getZ();
      }

      double gainScaleFactor = 1.0;

      if (!transitionToFallStartTime.isNaN())
      {
         double ratio = (currentTime.getValue() - transitionToFallStartTime.getValue()) / transitionDurationToFall.getValue();
         fallActivationRatio.set(MathTools.clamp(ratio, 0.0, 1.0));
         gainScaleFactor = 1.0 - fallActivationRatio.getValue();
      }

      // TODO is setting the support side the same thing as passing in the support side?

      if (isInDoubleSupport)
      {
         OneDoFJointBasics leftKnee = kneeJoints.get(RobotSide.LEFT);
         OneDoFJointBasics rightKnee = kneeJoints.get(RobotSide.RIGHT);

         double leftJointAngle = leftKnee.getQ();
         double rightJointAngle = rightKnee.getQ();

         if (leftJointAngle < rightJointAngle)
            kneeSideToControl.set(RobotSide.LEFT);
         else
            kneeSideToControl.set(RobotSide.RIGHT);

         hackStraightestKneeAngle.set(Math.min(leftJointAngle, rightJointAngle));
      }
      else
      {
         OneDoFJointBasics supportKnee = kneeJoints.get(supportLegSide.getValue());
         hackStraightestKneeAngle.set(supportKnee.getQ());
         kneeSideToControl.set(supportLegSide.getValue());
      }

      double control = -hackKp.getValue() * (hackDesiredKneeAngle.getValue() - hackStraightestKneeAngle.getValue());
      control = EuclidCoreTools.clamp(control, 0.02);
      //      if (straightestKneeAngle > 0.35)
      //         zDesired = zCurrent + 0.1;
      //      else if (straightestKneeAngle < 0.25)
      //         zDesired = zCurrent - 0.1;
      //
      //      else
      double zDesired = zCurrent + control;

      //      kneeJoints.get(sup)

      hackZDesired.set(zDesired);
      hackZCurrent.set(zCurrent);

      desiredPosition.set(0.0, 0.0, zDesired);
      desiredVelocity.set(0.0, 0.0, 0.0);
      desiredAcceleration.set(0.0, 0.0, 0.0);

      updateGains(gainScaleFactor);
      pelvisHeightControlCommand.setInverseDynamics(desiredPosition, desiredVelocity, desiredAcceleration);

      OneDoFJointFeedbackControlCommand supportKneeControlCommand = kneeControlCommands.get(kneeSideToControl.getValue());
      supportKneeControlCommand.setWeightForSolver(10.0);
      supportKneeControlCommand.setInverseDynamics(desiredSupportKneeAngles.get(kneeSideToControl.getValue()).getValue(), 0.0, 0.0);

      OneDoFJointFeedbackControlCommand nonSupportKneeControlCommand = kneeControlCommands.get(kneeSideToControl.getValue().getOppositeSide());
      supportKneeControlCommand.setGains(kneeGains);
      nonSupportKneeControlCommand.setGains(kneeGains);

      if (isInDoubleSupport)
      {
         nonSupportKneeControlCommand.setWeightForSolver(1.0);
         nonSupportKneeControlCommand.setInverseDynamics(desiredSupportKneeAngles.get(kneeSideToControl.getValue().getOppositeSide()).getValue(), 0.0, 0.0);
      }
      else
      {
         nonSupportKneeControlCommand.setWeightForSolver(0.0);
         nonSupportKneeControlCommand.setInverseDynamics(1.2, 0.0, 0.0);
      }
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      FeedbackControlCommandList commandList = new FeedbackControlCommandList();
      commandList.addCommand(kneeControlCommands.get(RobotSide.LEFT));
      commandList.addCommand(kneeControlCommands.get(RobotSide.RIGHT));
      return commandList;
   }

   @Override
   public FeedbackControlCommand<?> getHeightControlCommand()
   {
      return pelvisHeightControlCommand;
   }

   @Override
   public boolean getControlHeightWithMomentum()
   {
      return controlHeightWithMomentum.getValue();
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   public void setGains(PDGainsReadOnly gains, DoubleProvider maximumComVelocity)
   {
      this.gains = gains;
   }

   private final DefaultPID3DGains gainsTemp = new DefaultPID3DGains();

   public void updateGains(double gainScaleFactor)
   {
      gainsTemp.setProportionalGains(0.0, 0.0, gainScaleFactor * gains.getKp());
      gainsTemp.setDerivativeGains(0.0, 0.0, gainScaleFactor * gains.getKd());
      gainsTemp.setMaxFeedbackAndFeedbackRate(gains.getMaximumFeedback(), gains.getMaximumFeedbackRate());

      pelvisHeightControlCommand.setGains(gainsTemp);
      pelvisHeightControlCommand.getGains().setMaxFeedbackAndFeedbackRate(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().setWeightFrames(null, worldFrame);
      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().setAngularWeights(0.0, 0.0, 0.0);
//      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().getLinearPart().set(pelvisTaskpaceFeedbackWeight);
   }

   @Override
   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return null;
   }
}
