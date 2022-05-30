package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
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
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class HeightThroughKneeControlState implements PelvisAndCenterOfMassHeightControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble hackKp = new YoDouble("hackKp", registry);
   private final YoDouble hackDesiredKneeAngle = new YoDouble("hackDesiredKneeAngle", registry);
   private final YoDouble hackStraightestKneeAngle = new YoDouble("hackStraightestKneeAngle", registry);
   private final YoDouble hackZDesired = new YoDouble("hackZDesired", registry);
   private final YoDouble hackZCurrent = new YoDouble("hackZCurrent", registry);
   private final YoEnum<RobotSide> kneeSideToControl = new YoEnum<>("kneeSideToControl", registry, RobotSide.class);
   private final YoEnum<RobotSide> supportLegSide = new YoEnum<>("kneeControlSupportLegSide", registry, RobotSide.class);

   private final MovingReferenceFrame pelvisFrame;

   private final FeedbackControlCommandList feedbackCommandList = new FeedbackControlCommandList();
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

      pelvisFrame = referenceFrames.getPelvisFrame();

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
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
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

   private final FramePoint3D pelvisPosition = new FramePoint3D();

   @Override
   public void computeCoMHeightCommand(FrameVector2DReadOnly desiredICPVelocity,
                                       FrameVector2DReadOnly desiredCoMVelocity,
                                       boolean isInDoubleSupport,
                                       double omega0,
                                       FeetManager feetManager)
   {
      pelvisPosition.setToZero(pelvisFrame);
      pelvisPosition.changeFrame(worldFrame);
      double zCurrent = pelvisPosition.getZ();

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
      control = MathTools.clamp(control, 0.02);
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
      desiredVelocity.setToZero();
      desiredAcceleration.setToZero();

      updateGains();
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
      feedbackCommandList.clear();
      feedbackCommandList.addCommand(kneeControlCommands.get(RobotSide.LEFT));
      feedbackCommandList.addCommand(kneeControlCommands.get(RobotSide.RIGHT));
      return feedbackCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getHeightControlCommand()
   {
      return pelvisHeightControlCommand;
   }

   @Override
   public boolean getControlHeightWithMomentum()
   {
      // FIXME this should be false?
      return true;
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   public void setGains(PDGainsReadOnly gains)
   {
      this.gains = gains;
   }

   private final DefaultPID3DGains gainsTemp = new DefaultPID3DGains();

   public void updateGains()
   {
      gainsTemp.setProportionalGains(0.0, 0.0, gains.getKp());
      gainsTemp.setDerivativeGains(0.0, 0.0, gains.getKd());
      gainsTemp.setMaxFeedbackAndFeedbackRate(gains.getMaximumFeedback(), gains.getMaximumFeedbackRate());

      pelvisHeightControlCommand.setGains(gainsTemp);
      pelvisHeightControlCommand.getGains().setMaxFeedbackAndFeedbackRate(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().setWeightFrames(null, worldFrame);
      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().setAngularWeights(0.0, 0.0, 0.0);
   }

   @Override
   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      return null;
   }
}
