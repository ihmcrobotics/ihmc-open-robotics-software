package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.TaskspaceTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.heightPlanning.CoMHeightPartialDerivativesDataBasics;
import us.ihmc.commonWalkingControlModules.heightPlanning.CoMHeightTimeDerivativesCalculator;
import us.ihmc.commonWalkingControlModules.heightPlanning.CoMHeightTimeDerivativesSmoother;
import us.ihmc.commonWalkingControlModules.heightPlanning.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.heightPlanning.YoCoMHeightPartialDerivativesData;
import us.ihmc.commonWalkingControlModules.heightPlanning.YoCoMHeightTimeDerivativesData;
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
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
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

   private final YoDouble hackKp = new YoDouble("hackKp", registry);
   private final YoDouble hackDesiredKneeAngle = new YoDouble("hackDesiredKneeAngle", registry);
   private final YoDouble hackStraightestKneeAngle = new YoDouble("hackStraightestKneeAngle", registry);
   private final YoDouble hackZDesired = new YoDouble("hackZDesired", registry);
   private final YoDouble hackZCurrent = new YoDouble("hackZCurrent", registry);
   private final YoEnum<RobotSide> kneeSideToControl = new YoEnum<RobotSide>("kneeSideToControl", registry, RobotSide.class);
  
   private final DoubleProvider currentTime;
   private final YoDouble transitionDurationToFall = new YoDouble("comHeightTransitionDurationToFall", registry);
   private final YoDouble transitionToFallStartTime = new YoDouble("comHeightTransitionToFallStartTime", registry);
   private final YoDouble fallActivationRatio = new YoDouble("comHeightFallActivationRatio", registry);
   private final DoubleProvider fallAccelerationMagnitude = new DoubleParameter("comHeightFallAccelerationMagnitude", registry, 5.0);

   private final ReferenceFrame centerOfMassFrame;
   private final CenterOfMassStateProvider centerOfMassStateProvider;
   private final MovingReferenceFrame pelvisFrame;
   private final LookAheadCoMHeightTrajectoryGenerator centerOfMassTrajectoryGenerator;

   private final FramePoint3D statusDesiredPosition = new FramePoint3D();
   private final FramePoint3D statusActualPosition = new FramePoint3D();
   private final TaskspaceTrajectoryStatusMessageHelper statusHelper = new TaskspaceTrajectoryStatusMessageHelper("pelvisHeight");

   private final PointFeedbackControlCommand pelvisHeightControlCommand = new PointFeedbackControlCommand();
//   private final CenterOfMassFeedbackControlCommand comHeightControlCommand = new CenterOfMassFeedbackControlCommand();
   private final SideDependentList<OneDoFJointFeedbackControlCommand> kneeControlCommands;
   
   private Vector3DReadOnly pelvisTaskpaceFeedbackWeight;
   private final FullHumanoidRobotModel fullRobotModel;

   private final SideDependentList<OneDoFJointBasics> kneeJoints;
   private final YoDouble leftDesiredSupportKneeAngle = new YoDouble("leftDesiredSupportKneeAngle", registry);
   private final YoDouble rightDesiredSupportKneeAngle = new YoDouble("rightDesiredSupportKneeAngle", registry);
   private final SideDependentList<YoDouble> desiredSupportKneeAngles = new SideDependentList<YoDouble>(leftDesiredSupportKneeAngle, rightDesiredSupportKneeAngle);

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
      centerOfMassStateProvider = controllerToolbox;
      pelvisFrame = referenceFrames.getPelvisFrame();

      centerOfMassTrajectoryGenerator = createTrajectoryGenerator(controllerToolbox, walkingControllerParameters, referenceFrames);

      // TODO: Fix low level stuff so that we are truly controlling pelvis height and not CoM height.
      controlPelvisHeightInsteadOfCoMHeight.set(walkingControllerParameters.controlPelvisHeightInsteadOfCoMHeight());
      controlHeightWithMomentum.set(walkingControllerParameters.controlHeightWithMomentum());

      double controlDT = controllerToolbox.getControlDT();
      comHeightTimeDerivativesSmoother = new CoMHeightTimeDerivativesSmoother(controlDT, registry);

      currentTime = controllerToolbox.getYoTime();

      SelectionMatrix3D selectionMatrix = new SelectionMatrix3D(worldFrame, false, false, true);
      pelvisHeightControlCommand.set(fullRobotModel.getElevator(), fullRobotModel.getPelvis());
      FramePoint3D pelvisPoint = new FramePoint3D(pelvisFrame);
      pelvisPoint.set(0.0, 0.0, 0.0);
      pelvisPoint.changeFrame(fullRobotModel.getPelvis().getBodyFixedFrame());
      pelvisHeightControlCommand.setBodyFixedPointToControl(pelvisPoint);
      pelvisHeightControlCommand.setSelectionMatrix(selectionMatrix);
//      comHeightControlCommand.setSelectionMatrix(selectionMatrix);

      OneDoFJointFeedbackControlCommand leftKneeControlCommand = new OneDoFJointFeedbackControlCommand();
      OneDoFJointFeedbackControlCommand rightKneeControlCommand = new OneDoFJointFeedbackControlCommand();

      leftKneeControlCommand.setJoint(leftKneePitch);
      rightKneeControlCommand.setJoint(rightKneePitch);

      leftKneeControlCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      rightKneeControlCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

      leftKneeControlCommand.setWeightForSolver(10.0);
      rightKneeControlCommand.setWeightForSolver(10.0);
      
      YoPDGains kneeGains = new YoPDGains("kneeGains", registry);
      kneeGains.setKp(100.0);
      kneeGains.setKd(GainCalculator.computeDerivativeGain(100.0, 0.7));
//      kneeGains.setZeta(0.7);

      leftKneeControlCommand.setGains(kneeGains);
      rightKneeControlCommand.setGains(kneeGains);

      kneeControlCommands = new SideDependentList<>(leftKneeControlCommand, rightKneeControlCommand);

      parentRegistry.addChild(registry);
   }

   public LookAheadCoMHeightTrajectoryGenerator createTrajectoryGenerator(HighLevelHumanoidControllerToolbox controllerToolbox,
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

      return new LookAheadCoMHeightTrajectoryGenerator(minimumHeightAboveGround,
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
      transitionToFallStartTime.setToNaN();
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
      centerOfMassTrajectoryGenerator.initializeDesiredHeightToCurrent();
      comHeightTimeDerivativesSmoother.reset();
      transitionToFallStartTime.setToNaN();
   }

   public void initializeToNominalDesiredHeight()
   {
      centerOfMassTrajectoryGenerator.initializeToNominalHeight();
   }

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      if (!transitionToFallStartTime.isNaN())
         initialize();
      centerOfMassTrajectoryGenerator.initialize(transferToAndNextFootstepsData, extraToeOffHeight);
   }

   public void initializeTransitionToFall(double transitionDuration)
   {
      if (transitionToFallStartTime.isNaN())
      {
         transitionToFallStartTime.set(currentTime.getValue());
         transitionDurationToFall.set(transitionDuration);
      }
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

   public void setWeights(Vector3DReadOnly weight)
   {
      this.pelvisTaskpaceFeedbackWeight = weight;
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
                                       RobotSide supportSide,
                                       double omega0,
                                       FeetManager feetManager)
   {
      solve(comHeightPartialDerivatives, isInDoubleSupport);
      statusHelper.updateWithTimeInTrajectory(centerOfMassTrajectoryGenerator.getOffsetHeightTimeInTrajectory());

      comPosition.setToZero(centerOfMassFrame);
      comVelocity.setIncludingFrame(centerOfMassStateProvider.getCenterOfMassVelocity());
      comPosition.changeFrame(worldFrame);
      comVelocity.changeFrame(worldFrame);

      double zCurrent = comPosition.getZ();

      // FIXME should we be using the desired values instead of the current values?
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
                                                                         desiredCoMVelocity,
                                                                         desiredComAcceleration,
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

      double gainScaleFactor = 1.0;

      if (!transitionToFallStartTime.isNaN())
      {
         double ratio = (currentTime.getValue() - transitionToFallStartTime.getValue()) / transitionDurationToFall.getValue();
         fallActivationRatio.set(MathTools.clamp(ratio, 0.0, 1.0));
         zddFeedForward = EuclidCoreTools.interpolate(zddFeedForward, -fallAccelerationMagnitude.getValue(), fallActivationRatio.getValue());
         gainScaleFactor = 1.0 - fallActivationRatio.getValue();
      }

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
         OneDoFJointBasics supportKnee = kneeJoints.get(supportSide);
         hackStraightestKneeAngle.set(supportKnee.getQ());
         kneeSideToControl.set(supportSide);
      }

      double control = -hackKp.getValue() * (hackDesiredKneeAngle.getValue() - hackStraightestKneeAngle.getValue());
      control = EuclidCoreTools.clamp(control, 0.02);
      //      if (straightestKneeAngle > 0.35)
      //         zDesired = zCurrent + 0.1;
      //      else if (straightestKneeAngle < 0.25)
      //         zDesired = zCurrent - 0.1;
      //
      //      else 
      zDesired = zCurrent + control;

      //      kneeJoints.get(sup)

      hackZDesired.set(zDesired);
      hackZCurrent.set(zCurrent);

      desiredPosition.set(0.0, 0.0, zDesired);
      desiredVelocity.set(0.0, 0.0, 0.0); //zdDesired);
      desiredAcceleration.set(0.0, 0.0, 0.0);//zddFeedForward);

      updateGains(gainScaleFactor);
      pelvisHeightControlCommand.setInverseDynamics(desiredPosition, desiredVelocity, desiredAcceleration);
//      comHeightControlCommand.setInverseDynamics(desiredPosition, desiredVelocity, desiredAcceleration);
      
      OneDoFJointFeedbackControlCommand supportKneeControlCommand = kneeControlCommands.get(kneeSideToControl.getValue());
      supportKneeControlCommand.setWeightForSolver(10.0);
//      supportKneeControlCommand.setInverseDynamics(kneeJoints.get(kneeSideToControl.getValue()).getQ()+0.01, kneeJoints.get(kneeSideToControl.getValue()).getQ(), 0.0);
//      supportKneeControlCommand.setInverseDynamics(0.4, kneeJoints.get(kneeSideToControl.getValue()).getQd(), 0.0);
      supportKneeControlCommand.setInverseDynamics(desiredSupportKneeAngles.get(kneeSideToControl.getValue()).getValue(), 0.0, 0.0);
      
      OneDoFJointFeedbackControlCommand nonSupportKneeControlCommand = kneeControlCommands.get(kneeSideToControl.getValue().getOppositeSide());
//      nonSupportKneeControlCommand.setInverseDynamics(kneeJoints.get(kneeSideToControl.getValue().getOppositeSide()).getQ()+0.01, kneeJoints.get(kneeSideToControl.getValue().getOppositeSide()).getQd(), 0.0);
//      nonSupportKneeControlCommand.setInverseDynamics(0.4, kneeJoints.get(kneeSideToControl.getValue().getOppositeSide()).getQd(), 0.0);
      
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

   
   /**
    * This is what goes to the controller core.
    */
   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      FeedbackControlCommandList commandList = new FeedbackControlCommandList();
//      commandList.addCommand(pelvisHeightControlCommand);
//      commandList.addCommand(kneeControlCommands.get(kneeSideToControl.getValue()));
      commandList.addCommand(kneeControlCommands.get(RobotSide.LEFT));
      commandList.addCommand(kneeControlCommands.get(RobotSide.RIGHT));
      return commandList;
      
//      if (controlHeightWithMomentum.getValue() || !controlPelvisHeightInsteadOfCoMHeight.getValue())
//         return null;
//      else
//         return pelvisHeightControlCommand;
      
      
//      pelvisHeightControlCommand.setWeightForSolver(0.0);
//      return pelvisHeightControlCommand;

//      return kneeControlCommands.get(kneeSideToControl.getValue());
   }

   /**
    * This is what goes to the BalanceManager in WalkingHighLevelHumanoidController
    */
   @Override
   public FeedbackControlCommand<?> getHeightControlCommand()
   {
      FeedbackControlCommandList commandList = new FeedbackControlCommandList();
      commandList.addCommand(pelvisHeightControlCommand);
//      commandList.addCommand(kneeControlCommands.get(RobotSide.LEFT));
//      commandList.addCommand(kneeControlCommands.get(RobotSide.RIGHT));
      return commandList;
      
//      if (controlPelvisHeightInsteadOfCoMHeight.getBooleanValue())
//         return pelvisHeightControlCommand;
//      else
//         return comHeightControlCommand;
      
//      pelvisHeightControlCommand.setWeightForSolver(0.0);
//      return pelvisHeightControlCommand;
      
//      return kneeControlCommands.get(kneeSideToControl.getValue());

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
      comHeightTimeDerivativesSmoother.setGains(gains, maximumComVelocity);
   }

   private final DefaultPID3DGains gainsTemp = new DefaultPID3DGains();

   public void updateGains(double gainScaleFactor)
   {
      gainsTemp.setProportionalGains(0.0, 0.0, gainScaleFactor * gains.getKp());
      gainsTemp.setDerivativeGains(0.0, 0.0, gainScaleFactor * gains.getKd());
      gainsTemp.setMaxFeedbackAndFeedbackRate(gains.getMaximumFeedback(), gains.getMaximumFeedbackRate());

//      comHeightControlCommand.setGains(gainsTemp);
      pelvisHeightControlCommand.setGains(gainsTemp);
      pelvisHeightControlCommand.getGains().setMaxFeedbackAndFeedbackRate(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().setWeightFrames(null, worldFrame);
      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().setAngularWeights(0.0, 0.0, 0.0);
      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().getLinearPart().set(pelvisTaskpaceFeedbackWeight);
//      pelvisHeightControlCommand.getSpatialAccelerationCommand().getWeightMatrix().getLinearPart().setWeights(0.0, 0.0, 0.0);
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
