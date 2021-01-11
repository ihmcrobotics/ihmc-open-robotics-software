package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.TaskspaceTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.heightPlanning.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.PDControllerWithGainSetter;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class CenterOfMassHeightControlState implements PelvisAndCenterOfMassHeightControlState
{

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean controlPelvisHeightInsteadOfCoMHeight = new YoBoolean("controlPelvisHeightInsteadOfCoMHeight", registry);

   private final CoMHeightTimeDerivativesSmoother comHeightTimeDerivativesSmoother;
   private final YoDouble desiredCoMHeightFromTrajectory = new YoDouble("desiredCoMHeightFromTrajectory", registry);
   private final YoDouble desiredCoMHeightVelocityFromTrajectory = new YoDouble("desiredCoMHeightVelocityFromTrajectory", registry);
   private final YoDouble desiredCoMHeightAccelerationFromTrajectory = new YoDouble("desiredCoMHeightAccelerationFromTrajectory", registry);
   private final YoDouble desiredCoMHeightJerkFromTrajectory = new YoDouble("desiredCoMHeightJerkFromTrajectory", registry);
   //   private final YoDouble desiredCoMHeightBeforeSmoothing = new YoDouble("desiredCoMHeightBeforeSmoothing", registry);
   //   private final YoDouble desiredCoMHeightVelocityBeforeSmoothing = new YoDouble("desiredCoMHeightVelocityBeforeSmoothing", registry);
   //   private final YoDouble desiredCoMHeightAccelerationBeforeSmoothing = new YoDouble("desiredCoMHeightAccelerationBeforeSmoothing", registry);
   private final YoDouble desiredCoMHeightCorrected = new YoDouble("desiredCoMHeightCorrected", registry);
   private final YoDouble desiredCoMHeightVelocityCorrected = new YoDouble("desiredCoMHeightVelocityCorrected", registry);
   private final YoDouble desiredCoMHeightAccelerationCorrected = new YoDouble("desiredCoMHeightAccelerationCorrected", registry);
   private final YoDouble desiredCoMHeightAfterSmoothing = new YoDouble("desiredCoMHeightAfterSmoothing", registry);
   private final YoDouble desiredCoMHeightVelocityAfterSmoothing = new YoDouble("desiredCoMHeightVelocityAfterSmoothing", registry);
   private final YoDouble desiredCoMHeightAccelerationAfterSmoothing = new YoDouble("desiredCoMHeightAccelerationAfterSmoothing", registry);
   private final YoDouble desiredCoMHeightJerkAfterSmoothing = new YoDouble("desiredCoMHeightJerkAfterSmoothing", registry);

   private final ReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final MovingReferenceFrame pelvisFrame;
   private final BetterLookAheadCoMHeightTrajectoryGenerator centerOfMassTrajectoryGenerator;

   private final FramePoint3D statusDesiredPosition = new FramePoint3D();
   private final FramePoint3D statusActualPosition = new FramePoint3D();
   private final TaskspaceTrajectoryStatusMessageHelper statusHelper = new TaskspaceTrajectoryStatusMessageHelper("pelvisHeight");

   private final PointFeedbackControlCommand pelvisHeightControlCommand = new PointFeedbackControlCommand();
   private final CenterOfMassFeedbackControlCommand comHeightControlCommand = new CenterOfMassFeedbackControlCommand();

   public CenterOfMassHeightControlState(HighLevelHumanoidControllerToolbox controllerToolbox,
                                         WalkingControllerParameters walkingControllerParameters,
                                         YoRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      centerOfMassJacobian = controllerToolbox.getCenterOfMassJacobian();
      pelvisFrame = referenceFrames.getPelvisFrame();

      centerOfMassTrajectoryGenerator = createTrajectoryGenerator(controllerToolbox, walkingControllerParameters, referenceFrames);

      // TODO: Fix low level stuff so that we are truly controlling pelvis height and not CoM height.
      controlPelvisHeightInsteadOfCoMHeight.set(true);

      double controlDT = controllerToolbox.getControlDT();
      comHeightTimeDerivativesSmoother = new CoMHeightTimeDerivativesSmoother(controlDT, registry);

      SelectionMatrix3D selectionMatrix = new SelectionMatrix3D(worldFrame, false, false, true);
      pelvisHeightControlCommand.set(fullRobotModel.getElevator(), fullRobotModel.getPelvis());
      FramePoint3D pelvisPoint = new FramePoint3D(pelvisFrame);
      pelvisPoint.changeFrame(fullRobotModel.getPelvis().getBodyFixedFrame());
      pelvisHeightControlCommand.setBodyFixedPointToControl(pelvisPoint);
      pelvisHeightControlCommand.setSelectionMatrix(selectionMatrix);
      comHeightControlCommand.setSelectionMatrix(selectionMatrix);

      parentRegistry.addChild(registry);
   }

   public BetterLookAheadCoMHeightTrajectoryGenerator createTrajectoryGenerator(HighLevelHumanoidControllerToolbox controllerToolbox,
                                                                                WalkingControllerParameters walkingControllerParameters,
                                                                                CommonHumanoidReferenceFrames referenceFrames)
   {
      double ankleToGround = Double.NEGATIVE_INFINITY;
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = controllerToolbox.getFullRobotModel().getFoot(robotSide);

         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         RigidBodyTransform ankleToSole = new RigidBodyTransform();
         ankleFrame.getTransformToDesiredFrame(ankleToSole, soleFrame);
         ankleToGround = Math.max(ankleToGround, Math.abs(ankleToSole.getTranslationZ()));
      }

      FramePoint3D leftHipPitch = new FramePoint3D(controllerToolbox.getFullRobotModel().getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint());
      FramePoint3D rightHipPitch = new FramePoint3D(controllerToolbox.getFullRobotModel().getLegJoint(RobotSide.RIGHT, LegJointName.HIP_PITCH).getFrameAfterJoint());
      leftHipPitch.changeFrame(controllerToolbox.getPelvisZUpFrame());
      rightHipPitch.changeFrame(controllerToolbox.getPelvisZUpFrame());

      double hipWidth = leftHipPitch.getY() - rightHipPitch.getY();


      double minimumHeightAboveGround = walkingControllerParameters.minimumHeightAboveAnkle() + ankleToGround;
      double nominalHeightAboveGround = walkingControllerParameters.nominalHeightAboveAnkle() + ankleToGround;
      double maximumHeightAboveGround = walkingControllerParameters.maximumHeightAboveAnkle() + ankleToGround;
      double defaultOffsetHeightAboveGround = walkingControllerParameters.defaultOffsetHeightAboveAnkle();

      double doubleSupportPercentageIn = 0.3;

      return new BetterLookAheadCoMHeightTrajectoryGenerator(minimumHeightAboveGround,
                                                             nominalHeightAboveGround,
                                                             maximumHeightAboveGround,
                                                             defaultOffsetHeightAboveGround,
                                                             doubleSupportPercentageIn,
                                                             hipWidth,
                                                             centerOfMassFrame,
                                                             pelvisFrame,
                                                             referenceFrames.getSoleZUpFrames(),
                                                             controllerToolbox.getYoTime(),
                                                             controllerToolbox.getYoGraphicsListRegistry(),
                                                             registry);
   }


   @Override
   public void initialize()
   {
      centerOfMassTrajectoryGenerator.reset();
      comHeightTimeDerivativesSmoother.reset();
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
      centerOfMassTrajectoryGenerator.initializeDesiredHeightToCurrent();
      comHeightTimeDerivativesSmoother.reset();
   }

   public void initialize(NewTransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      centerOfMassTrajectoryGenerator.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
   }

   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      if (centerOfMassTrajectoryGenerator.handlePelvisTrajectoryCommand(command))
      {
         SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();
         se3Trajectory.setSequenceId(command.getSequenceId());
         statusHelper.registerNewTrajectory(se3Trajectory);
      }
   }

   public void handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      if (centerOfMassTrajectoryGenerator.handlePelvisHeightTrajectoryCommand(command))
      {
         EuclideanTrajectoryControllerCommand euclideanTrajectory = command.getEuclideanTrajectory();
         euclideanTrajectory.setSequenceId(command.getSequenceId());
         statusHelper.registerNewTrajectory(euclideanTrajectory);
      }
   }

   @Override
   public void goHome(double trajectoryTime)
   {
      centerOfMassTrajectoryGenerator.goHome(trajectoryTime);
   }

   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      centerOfMassTrajectoryGenerator.handleStopAllTrajectoryCommand(command);
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      centerOfMassTrajectoryGenerator.setSupportLeg(supportLeg);
   }

   private void solve(CoMHeightPartialDerivativesDataBasics comHeightPartialDerivativesToPack, boolean isInDoubleSupport)
   {
      centerOfMassTrajectoryGenerator.solve(comHeightPartialDerivativesToPack, isInDoubleSupport);
   }

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D desiredAcceleration = new FrameVector3D();
   private PDGainsReadOnly gains;
   // Temporary objects to reduce garbage collection.
   private final CoMHeightPartialDerivativesDataBasics comHeightPartialDerivatives = new YoCoMHeightPartialDerivativesData(registry);
   private final FramePoint3D comPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D(worldFrame);
   private final FrameVector2D comXYVelocity = new FrameVector2D();
   private final FrameVector2D desiredComAcceleration = new FrameVector2D();
   private final YoCoMHeightTimeDerivativesData comHeightDataBeforeSmoothing = new YoCoMHeightTimeDerivativesData("beforeSmoothing", registry);
   private final YoCoMHeightTimeDerivativesData comHeightDataAfterSmoothing = new YoCoMHeightTimeDerivativesData("afterSmoothing", registry);
   private final YoCoMHeightTimeDerivativesData comHeightDataAfterSingularityAvoidance = new YoCoMHeightTimeDerivativesData("afterSingularityAvoidance", registry);
   private final YoCoMHeightTimeDerivativesData comHeightDataAfterUnreachableFootstep = new YoCoMHeightTimeDerivativesData("afterUnreachableFootstep", registry);
   private final YoCoMHeightTimeDerivativesData finalComHeightData = new YoCoMHeightTimeDerivativesData("finalComHeightData", registry);

   private final FramePoint3D desiredCenterOfMassHeightPoint = new FramePoint3D(worldFrame);
   private final FramePoint3D pelvisPosition = new FramePoint3D();
   private final Twist currentPelvisTwist = new Twist();

   private boolean desiredCMPcontainedNaN = false;

   @Override
   public void computeCoMHeightCommand(FrameVector2DReadOnly desiredICPVelocity,
                                       FrameVector2DReadOnly desiredCoMVelocity,
                                       boolean isInDoubleSupport,
                                       double omega0,
                                       boolean isRecoveringFromPush,
                                       FeetManager feetManager)
   {
      solve(comHeightPartialDerivatives, isInDoubleSupport);
      statusHelper.updateWithTimeInTrajectory(centerOfMassTrajectoryGenerator.getOffsetHeightTimeInTrajectory());

      comPosition.setToZero(centerOfMassFrame);
      comVelocity.setIncludingFrame(centerOfMassJacobian.getCenterOfMassVelocity());
      comPosition.changeFrame(worldFrame);
      comVelocity.changeFrame(worldFrame);

      double zCurrent = comPosition.getZ();

      if (controlPelvisHeightInsteadOfCoMHeight.getBooleanValue())
      {
         pelvisPosition.setToZero(pelvisFrame);
         pelvisPosition.changeFrame(worldFrame);
         zCurrent = pelvisPosition.getZ();
         pelvisFrame.getTwistOfFrame(currentPelvisTwist);
         currentPelvisTwist.changeFrame(worldFrame);
      }

      // TODO: use current omega0 instead of previous
      comXYVelocity.setIncludingFrame(comVelocity);
      if (desiredCoMVelocity.containsNaN())
      {
         if (!desiredCMPcontainedNaN)
            LogTools.error("Desired CMP containes NaN, setting it to the ICP - only showing this error once");
         desiredComAcceleration.setToZero(desiredICPVelocity.getReferenceFrame());
         desiredCMPcontainedNaN = true;
      }
      else
      {
         desiredComAcceleration.setIncludingFrame(desiredICPVelocity);
         desiredCMPcontainedNaN = false;
      }
      desiredComAcceleration.sub(desiredCoMVelocity);
      desiredComAcceleration.scale(omega0); // MathTools.square(omega0.getDoubleValue()) * (com.getX() - copX);

      CoMHeightTimeDerivativesCalculator.computeCoMHeightTimeDerivatives(comHeightDataBeforeSmoothing,
                                                                         desiredCoMVelocity, desiredComAcceleration,
                                                                         comHeightPartialDerivatives);

      comHeightDataBeforeSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightFromTrajectory.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityFromTrajectory.set(comHeightDataBeforeSmoothing.getComHeightVelocity());
      desiredCoMHeightAccelerationFromTrajectory.set(comHeightDataBeforeSmoothing.getComHeightAcceleration());
      desiredCoMHeightJerkFromTrajectory.set(comHeightDataBeforeSmoothing.getComHeightJerk());

      //    correctCoMHeight(desiredICPVelocity, zCurrent, comHeightDataBeforeSmoothing, false, false);

      //    comHeightDataBeforeSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      //    desiredCoMHeightBeforeSmoothing.set(desiredCenterOfMassHeightPoint.getZ());
      //    desiredCoMHeightVelocityBeforeSmoothing.set(comHeightDataBeforeSmoothing.getComHeightVelocity());
      //    desiredCoMHeightAccelerationBeforeSmoothing.set(comHeightDataBeforeSmoothing.getComHeightAcceleration());

      comHeightTimeDerivativesSmoother.smooth(comHeightDataAfterSmoothing, comHeightDataBeforeSmoothing);

      comHeightDataAfterSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightAfterSmoothing.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityAfterSmoothing.set(comHeightDataAfterSmoothing.getComHeightVelocity());
      desiredCoMHeightAccelerationAfterSmoothing.set(comHeightDataAfterSmoothing.getComHeightAcceleration());
      desiredCoMHeightJerkAfterSmoothing.set(comHeightDataBeforeSmoothing.getComHeightJerk());

      if (feetManager != null)
      {
         comHeightDataAfterSingularityAvoidance.set(comHeightDataAfterSmoothing);
         feetManager.correctCoMHeightForSupportSingularityAvoidance(zCurrent, comHeightDataAfterSingularityAvoidance);

         comHeightDataAfterUnreachableFootstep.set(comHeightDataAfterSingularityAvoidance);
         feetManager.correctCoMHeightForUnreachableFootstep(comHeightDataAfterUnreachableFootstep);

         finalComHeightData.set(comHeightDataAfterUnreachableFootstep);
      }
      else
      {
         finalComHeightData.set(comHeightDataAfterSmoothing);
      }

      finalComHeightData.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightCorrected.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityCorrected.set(finalComHeightData.getComHeightVelocity());
      desiredCoMHeightAccelerationCorrected.set(finalComHeightData.getComHeightAcceleration());

      double zDesired = desiredCenterOfMassHeightPoint.getZ();
      double zdDesired = finalComHeightData.getComHeightVelocity();
      double zddFeedForward = finalComHeightData.getComHeightAcceleration();

      desiredPosition.set(0.0, 0.0, zDesired);
      desiredVelocity.set(0.0, 0.0, zdDesired);
      desiredAcceleration.set(0.0, 0.0, zddFeedForward);

      updateGains();
//      pelvisHeightControlCommand.setBodyFixedPointToControl(new FramePoint3D(pelvisFrame));
      pelvisHeightControlCommand.setInverseDynamics(desiredPosition, desiredVelocity, desiredAcceleration);
      comHeightControlCommand.setInverseDynamics(desiredPosition, desiredVelocity, desiredAcceleration);
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getHeightControlCommand()
   {
      if (controlPelvisHeightInsteadOfCoMHeight.getBooleanValue())
         return pelvisHeightControlCommand;
      else
         return comHeightControlCommand;
   }

   @Override
   public void doAction(double timeInState)
   {
   }


   public void setGains(PDGainsReadOnly gains, DoubleProvider maximumComVelocity)
   {
      this.gains = gains;
      comHeightTimeDerivativesSmoother.setGains(gains, maximumComVelocity);
   }

   private final DefaultPID3DGains gainsTemp = new DefaultPID3DGains();

   public void updateGains()
   {
      gainsTemp.setProportionalGains(0.0, 0.0, gains.getKp());
      gainsTemp.setDerivativeGains(0.0, 0.0, gains.getKd());
      gainsTemp.setMaxFeedbackAndFeedbackRate(gains.getMaximumFeedback(), gains.getMaximumFeedbackRate());

      pelvisHeightControlCommand.setGains(gainsTemp);
      comHeightControlCommand.setGains(gainsTemp);
   }

   @Override
   public TaskspaceTrajectoryStatusMessage pollStatusToReport()
   {
      centerOfMassTrajectoryGenerator.getCurrentDesiredHeight(statusDesiredPosition);
      statusDesiredPosition.changeFrame(ReferenceFrame.getWorldFrame());
      statusDesiredPosition.setX(Double.NaN);
      statusDesiredPosition.setY(Double.NaN);
      if (controlPelvisHeightInsteadOfCoMHeight.getValue())
         statusActualPosition.setIncludingFrame(pelvisPosition);
      else
         statusActualPosition.setIncludingFrame(comPosition);
      statusActualPosition.setX(Double.NaN);
      statusActualPosition.setY(Double.NaN);

      return statusHelper.pollStatusMessage(statusDesiredPosition, statusActualPosition);
   }
}
