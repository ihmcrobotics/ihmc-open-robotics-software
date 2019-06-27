package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.TaskspaceTrajectoryStatusMessageHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightPartialDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesSmoother;
import us.ihmc.commonWalkingControlModules.trajectories.CoMXYTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.controllers.PDControllerWithGainSetter;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class CenterOfMassHeightControlState implements PelvisAndCenterOfMassHeightControlState
{

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean controlPelvisHeightInsteadOfCoMHeight = new YoBoolean("controlPelvisHeightInsteadOfCoMHeight", registry);

   private final CoMHeightTimeDerivativesCalculator coMHeightTimeDerivativesCalculator = new CoMHeightTimeDerivativesCalculator();
   private final CoMHeightTimeDerivativesSmoother coMHeightTimeDerivativesSmoother;
   private final YoDouble desiredCoMHeightFromTrajectory = new YoDouble("desiredCoMHeightFromTrajectory", registry);
   private final YoDouble desiredCoMHeightVelocityFromTrajectory = new YoDouble("desiredCoMHeightVelocityFromTrajectory", registry);
   private final YoDouble desiredCoMHeightAccelerationFromTrajectory = new YoDouble("desiredCoMHeightAccelerationFromTrajectory", registry);
   //   private final YoDouble desiredCoMHeightBeforeSmoothing = new YoDouble("desiredCoMHeightBeforeSmoothing", registry);
   //   private final YoDouble desiredCoMHeightVelocityBeforeSmoothing = new YoDouble("desiredCoMHeightVelocityBeforeSmoothing", registry);
   //   private final YoDouble desiredCoMHeightAccelerationBeforeSmoothing = new YoDouble("desiredCoMHeightAccelerationBeforeSmoothing", registry);
   private final YoDouble desiredCoMHeightCorrected = new YoDouble("desiredCoMHeightCorrected", registry);
   private final YoDouble desiredCoMHeightVelocityCorrected = new YoDouble("desiredCoMHeightVelocityCorrected", registry);
   private final YoDouble desiredCoMHeightAccelerationCorrected = new YoDouble("desiredCoMHeightAccelerationCorrected", registry);
   private final YoDouble desiredCoMHeightAfterSmoothing = new YoDouble("desiredCoMHeightAfterSmoothing", registry);
   private final YoDouble desiredCoMHeightVelocityAfterSmoothing = new YoDouble("desiredCoMHeightVelocityAfterSmoothing", registry);
   private final YoDouble desiredCoMHeightAccelerationAfterSmoothing = new YoDouble("desiredCoMHeightAccelerationAfterSmoothing", registry);

   private final YoDouble zDesired = new YoDouble("zDesired", registry);
   private final YoDouble zdDesired = new YoDouble("zdDesired", registry);
   private final YoDouble zddDesired = new YoDouble("zddDesired", registry);

   private final PDControllerWithGainSetter centerOfMassHeightController;

   private final ReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final ReferenceFrame pelvisFrame;
   private final LookAheadCoMHeightTrajectoryGenerator centerOfMassTrajectoryGenerator;

   private final double gravity;
   private final RigidBodyBasics pelvis;

   private final FramePoint3D statusDesiredPosition = new FramePoint3D();
   private final FramePoint3D statusActualPosition = new FramePoint3D();
   private final TaskspaceTrajectoryStatusMessageHelper statusHelper = new TaskspaceTrajectoryStatusMessageHelper("pelvisHeight");

   public CenterOfMassHeightControlState(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
                                         YoVariableRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      centerOfMassJacobian = new CenterOfMassJacobian(controllerToolbox.getFullRobotModel().getElevator(), worldFrame);
      pelvisFrame = referenceFrames.getPelvisFrame();

      gravity = controllerToolbox.getGravityZ();
      pelvis = controllerToolbox.getFullRobotModel().getPelvis();

      centerOfMassTrajectoryGenerator = createTrajectoryGenerator(controllerToolbox, walkingControllerParameters, referenceFrames);

      // TODO: Fix low level stuff so that we are truly controlling pelvis height and not CoM height.
      controlPelvisHeightInsteadOfCoMHeight.set(true);

      double controlDT = controllerToolbox.getControlDT();
      coMHeightTimeDerivativesSmoother = new CoMHeightTimeDerivativesSmoother(controlDT, registry);
      centerOfMassHeightController = new PDControllerWithGainSetter("CoMHeight", registry);

      parentRegistry.addChild(registry);
   }

   public LookAheadCoMHeightTrajectoryGenerator createTrajectoryGenerator(HighLevelHumanoidControllerToolbox controllerToolbox,
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames)
   {
      SideDependentList<RigidBodyTransform> transformsFromAnkleToSole = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = controllerToolbox.getFullRobotModel().getFoot(robotSide);
         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         RigidBodyTransform ankleToSole = new RigidBodyTransform();
         ankleFrame.getTransformToDesiredFrame(ankleToSole, soleFrame);
         transformsFromAnkleToSole.put(robotSide, ankleToSole);
      }

      double minimumHeightAboveGround = walkingControllerParameters.minimumHeightAboveAnkle();
      double nominalHeightAboveGround = walkingControllerParameters.nominalHeightAboveAnkle();
      double maximumHeightAboveGround = walkingControllerParameters.maximumHeightAboveAnkle();
      double defaultOffsetHeightAboveGround = walkingControllerParameters.defaultOffsetHeightAboveAnkle();

      double doubleSupportPercentageIn = 0.3;
      boolean activateDriftCompensation = false;
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      SideDependentList<? extends ReferenceFrame> ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      YoDouble yoTime = controllerToolbox.getYoTime();
      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      LookAheadCoMHeightTrajectoryGenerator centerOfMassTrajectoryGenerator = new LookAheadCoMHeightTrajectoryGenerator(minimumHeightAboveGround,
            nominalHeightAboveGround, maximumHeightAboveGround, defaultOffsetHeightAboveGround, doubleSupportPercentageIn, centerOfMassFrame, pelvisFrame,
            ankleZUpFrames, transformsFromAnkleToSole, yoTime, yoGraphicsListRegistry, registry);
      centerOfMassTrajectoryGenerator.setCoMHeightDriftCompensation(activateDriftCompensation);
      return centerOfMassTrajectoryGenerator;
   }

   @Override
   public void initialize()
   {
      centerOfMassTrajectoryGenerator.reset();
      coMHeightTimeDerivativesSmoother.reset();
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
      centerOfMassTrajectoryGenerator.initializeDesiredHeightToCurrent();
      coMHeightTimeDerivativesSmoother.reset();
   }

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
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

   public boolean hasBeenInitializedWithNextStep()
   {
      return centerOfMassTrajectoryGenerator.hasBeenInitializedWithNextStep();
   }

   private void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesToPack, boolean isInDoubleSupport)
   {
      centerOfMassTrajectoryGenerator.solve(coMHeightPartialDerivativesToPack, isInDoubleSupport);
   }

   // Temporary objects to reduce garbage collection.
   private final CoMHeightPartialDerivativesData coMHeightPartialDerivatives = new CoMHeightPartialDerivativesData();
   private final FramePoint3D comPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D(worldFrame);
   private final FrameVector2D comXYVelocity = new FrameVector2D();
   private final FrameVector2D comXYAcceleration = new FrameVector2D();
   private final CoMHeightTimeDerivativesData comHeightDataBeforeSmoothing = new CoMHeightTimeDerivativesData("beforeSmoothing", registry);
   private final CoMHeightTimeDerivativesData comHeightDataAfterSmoothing = new CoMHeightTimeDerivativesData("afterSmoothing", registry);
   private final CoMHeightTimeDerivativesData comHeightDataAfterSingularityAvoidance = new CoMHeightTimeDerivativesData("afterSingularityAvoidance", registry);
   private final CoMHeightTimeDerivativesData comHeightDataAfterUnreachableFootstep = new CoMHeightTimeDerivativesData("afterUnreachableFootstep", registry);
   private final CoMHeightTimeDerivativesData finalComHeightData = new CoMHeightTimeDerivativesData("finalComHeightData", registry);

   private final CoMXYTimeDerivativesData comXYTimeDerivatives = new CoMXYTimeDerivativesData();
   private final FramePoint3D desiredCenterOfMassHeightPoint = new FramePoint3D(worldFrame);
   private final FramePoint3D pelvisPosition = new FramePoint3D();
   private final FramePoint2D comPositionAsFramePoint2d = new FramePoint2D();
   private final Twist currentPelvisTwist = new Twist();

   private boolean desiredCMPcontainedNaN = false;

   @Override
   public double computeDesiredCoMHeightAcceleration(FrameVector2D desiredICPVelocity, boolean isInDoubleSupport, double omega0, boolean isRecoveringFromPush,
         FeetManager feetManager)
   {
      solve(coMHeightPartialDerivatives, isInDoubleSupport);
      statusHelper.updateWithTimeInTrajectory(centerOfMassTrajectoryGenerator.getOffsetHeightTimeInTrajectory());

      comPosition.setToZero(centerOfMassFrame);
      centerOfMassJacobian.reset();
      comVelocity.setIncludingFrame(centerOfMassJacobian.getCenterOfMassVelocity());
      comPosition.changeFrame(worldFrame);
      comVelocity.changeFrame(worldFrame);

      double zCurrent = comPosition.getZ();
      double zdCurrent = comVelocity.getZ();

      if (controlPelvisHeightInsteadOfCoMHeight.getBooleanValue())
      {
         pelvisPosition.setToZero(pelvisFrame);
         pelvisPosition.changeFrame(worldFrame);
         zCurrent = pelvisPosition.getZ();
         pelvis.getBodyFixedFrame().getTwistOfFrame(currentPelvisTwist);
         currentPelvisTwist.changeFrame(worldFrame);
         zdCurrent = comVelocity.getZ(); // Just use com velocity for now for damping...
      }

      // TODO: use current omega0 instead of previous
      comXYVelocity.setIncludingFrame(comVelocity.getReferenceFrame(), comVelocity.getX(), comVelocity.getY());
      if (desiredICPVelocity.containsNaN())
      {
         if (!desiredCMPcontainedNaN)
            LogTools.error("Desired CMP containes NaN, setting it to the ICP - only showing this error once");
         comXYAcceleration.setToZero(desiredICPVelocity.getReferenceFrame());
         desiredCMPcontainedNaN = true;
      }
      else
      {
         comXYAcceleration.setIncludingFrame(desiredICPVelocity);
         desiredCMPcontainedNaN = false;
      }
      comXYAcceleration.sub(comXYVelocity);
      comXYAcceleration.scale(omega0); // MathTools.square(omega0.getDoubleValue()) * (com.getX() - copX);

      // FrameVector2d comd2dSquared = new FrameVector2d(comXYVelocity.getReferenceFrame(), comXYVelocity.getX() * comXYVelocity.getX(), comXYVelocity.getY() * comXYVelocity.getY());
      comPositionAsFramePoint2d.setIncludingFrame(comPosition);
      comXYTimeDerivatives.setCoMXYPosition(comPositionAsFramePoint2d);
      comXYTimeDerivatives.setCoMXYVelocity(comXYVelocity);
      comXYTimeDerivatives.setCoMXYAcceleration(comXYAcceleration);

      coMHeightTimeDerivativesCalculator.computeCoMHeightTimeDerivatives(comHeightDataBeforeSmoothing, comXYTimeDerivatives, coMHeightPartialDerivatives);

      comHeightDataBeforeSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightFromTrajectory.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityFromTrajectory.set(comHeightDataBeforeSmoothing.getComHeightVelocity());
      desiredCoMHeightAccelerationFromTrajectory.set(comHeightDataBeforeSmoothing.getComHeightAcceleration());

      //    correctCoMHeight(desiredICPVelocity, zCurrent, comHeightDataBeforeSmoothing, false, false);

      //    comHeightDataBeforeSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      //    desiredCoMHeightBeforeSmoothing.set(desiredCenterOfMassHeightPoint.getZ());
      //    desiredCoMHeightVelocityBeforeSmoothing.set(comHeightDataBeforeSmoothing.getComHeightVelocity());
      //    desiredCoMHeightAccelerationBeforeSmoothing.set(comHeightDataBeforeSmoothing.getComHeightAcceleration());

      coMHeightTimeDerivativesSmoother.smooth(comHeightDataAfterSmoothing, comHeightDataBeforeSmoothing);

      comHeightDataAfterSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightAfterSmoothing.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityAfterSmoothing.set(comHeightDataAfterSmoothing.getComHeightVelocity());
      desiredCoMHeightAccelerationAfterSmoothing.set(comHeightDataAfterSmoothing.getComHeightAcceleration());

      if (feetManager != null)
      {
         comHeightDataAfterSingularityAvoidance.set(comHeightDataAfterSmoothing);
         feetManager.correctCoMHeightForSingularityAvoidance(desiredICPVelocity, zCurrent, comHeightDataAfterSingularityAvoidance);

         comHeightDataAfterUnreachableFootstep.set(comHeightDataAfterSingularityAvoidance);
         feetManager.correctCoMHeightForUnreachableFootstep(desiredICPVelocity, zCurrent, comHeightDataAfterUnreachableFootstep);
         
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

      double zddDesired = centerOfMassHeightController.compute(zCurrent, zDesired, zdCurrent, zdDesired) + zddFeedForward;

      // In a recovering context, accelerating upwards is just gonna make the robot fall faster. We should even consider letting the robot fall a bit.
      if (isRecoveringFromPush)
         zddDesired = Math.min(0.0, zddDesired);

      double epsilon = 1e-12;
      zddDesired = MathTools.clamp(zddDesired, -gravity + epsilon, Double.POSITIVE_INFINITY);

      this.zDesired.set(zDesired);
      this.zdDesired.set(zdDesired);
      this.zddDesired.set(zddDesired);

      return zddDesired;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   public void setGains(PDGainsReadOnly gains, DoubleProvider maximumComVelocity)
   {
      centerOfMassHeightController.setGains(gains);
      coMHeightTimeDerivativesSmoother.setGains(gains, maximumComVelocity);
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
