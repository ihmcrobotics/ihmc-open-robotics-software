package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightPartialDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesSmoother;
import us.ihmc.commonWalkingControlModules.trajectories.CoMXYTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class CenterOfMassHeightManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable controlPelvisHeightInsteadOfCoMHeight = new BooleanYoVariable("controlPelvisHeightInsteadOfCoMHeight", registry);

   private final CoMHeightTimeDerivativesCalculator coMHeightTimeDerivativesCalculator = new CoMHeightTimeDerivativesCalculator();
   private final CoMHeightTimeDerivativesSmoother coMHeightTimeDerivativesSmoother;
   private final DoubleYoVariable desiredCoMHeightFromTrajectory = new DoubleYoVariable("desiredCoMHeightFromTrajectory", registry);
   private final DoubleYoVariable desiredCoMHeightVelocityFromTrajectory = new DoubleYoVariable("desiredCoMHeightVelocityFromTrajectory", registry);
   private final DoubleYoVariable desiredCoMHeightAccelerationFromTrajectory = new DoubleYoVariable("desiredCoMHeightAccelerationFromTrajectory", registry);
   //   private final DoubleYoVariable desiredCoMHeightBeforeSmoothing = new DoubleYoVariable("desiredCoMHeightBeforeSmoothing", registry);
   //   private final DoubleYoVariable desiredCoMHeightVelocityBeforeSmoothing = new DoubleYoVariable("desiredCoMHeightVelocityBeforeSmoothing", registry);
   //   private final DoubleYoVariable desiredCoMHeightAccelerationBeforeSmoothing = new DoubleYoVariable("desiredCoMHeightAccelerationBeforeSmoothing", registry);
   private final DoubleYoVariable desiredCoMHeightCorrected = new DoubleYoVariable("desiredCoMHeightCorrected", registry);
   private final DoubleYoVariable desiredCoMHeightVelocityCorrected = new DoubleYoVariable("desiredCoMHeightVelocityCorrected", registry);
   private final DoubleYoVariable desiredCoMHeightAccelerationCorrected = new DoubleYoVariable("desiredCoMHeightAccelerationCorrected", registry);
   private final DoubleYoVariable desiredCoMHeightAfterSmoothing = new DoubleYoVariable("desiredCoMHeightAfterSmoothing", registry);
   private final DoubleYoVariable desiredCoMHeightVelocityAfterSmoothing = new DoubleYoVariable("desiredCoMHeightVelocityAfterSmoothing", registry);
   private final DoubleYoVariable desiredCoMHeightAccelerationAfterSmoothing = new DoubleYoVariable("desiredCoMHeightAccelerationAfterSmoothing", registry);

   private final PDController centerOfMassHeightController;

   private final ReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final ReferenceFrame pelvisFrame;
   private final LookAheadCoMHeightTrajectoryGenerator centerOfMassTrajectoryGenerator;

   private final double gravity;
   private final RigidBody pelvis;
   private final TwistCalculator twistCalculator;

   public CenterOfMassHeightManager(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
         YoVariableRegistry parentRegistry)
   {
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      centerOfMassJacobian = controllerToolbox.getCenterOfMassJacobian();
      pelvisFrame = referenceFrames.getPelvisFrame();

      gravity = controllerToolbox.getGravityZ();
      pelvis = controllerToolbox.getFullRobotModel().getPelvis();
      twistCalculator = controllerToolbox.getTwistCalculator();

      centerOfMassTrajectoryGenerator = createTrajectoryGenerator(controllerToolbox, walkingControllerParameters, referenceFrames);

      // TODO: Fix low level stuff so that we are truly controlling pelvis height and not CoM height.
      controlPelvisHeightInsteadOfCoMHeight.set(true);

      YoPDGains comHeightControlGains = walkingControllerParameters.createCoMHeightControlGains(registry);
      DoubleYoVariable kpCoMHeight = comHeightControlGains.getYoKp();
      DoubleYoVariable kdCoMHeight = comHeightControlGains.getYoKd();
      DoubleYoVariable maxCoMHeightAcceleration = comHeightControlGains.getYoMaximumFeedback();
      DoubleYoVariable maxCoMHeightJerk = comHeightControlGains.getYoMaximumFeedbackRate();

      double controlDT = controllerToolbox.getControlDT();
      // TODO Need to extract the maximum velocity parameter.
      coMHeightTimeDerivativesSmoother = new CoMHeightTimeDerivativesSmoother(null, maxCoMHeightAcceleration, maxCoMHeightJerk, controlDT, registry);
      this.centerOfMassHeightController = new PDController(kpCoMHeight, kdCoMHeight, "comHeight", registry);

      parentRegistry.addChild(registry);
   }

   public LookAheadCoMHeightTrajectoryGenerator createTrajectoryGenerator(HighLevelHumanoidControllerToolbox controllerToolbox,
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames)
   {
      double minimumHeightAboveGround = walkingControllerParameters.minimumHeightAboveAnkle();
      double nominalHeightAboveGround = walkingControllerParameters.nominalHeightAboveAnkle();
      double maximumHeightAboveGround = walkingControllerParameters.maximumHeightAboveAnkle();
      double defaultOffsetHeightAboveGround = walkingControllerParameters.defaultOffsetHeightAboveAnkle();
      double doubleSupportPercentageIn = 0.3;
      boolean activateDriftCompensation = walkingControllerParameters.getCoMHeightDriftCompensation();
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      SideDependentList<ReferenceFrame> ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      DoubleYoVariable yoTime = controllerToolbox.getYoTime();
      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      LookAheadCoMHeightTrajectoryGenerator centerOfMassTrajectoryGenerator = new LookAheadCoMHeightTrajectoryGenerator(minimumHeightAboveGround,
            nominalHeightAboveGround, maximumHeightAboveGround, defaultOffsetHeightAboveGround, doubleSupportPercentageIn, centerOfMassFrame, pelvisFrame,
            ankleZUpFrames, yoTime, yoGraphicsListRegistry, registry);
      centerOfMassTrajectoryGenerator.setCoMHeightDriftCompensation(activateDriftCompensation);
      return centerOfMassTrajectoryGenerator;
   }

   public void setCoMHeightGains(double kp, double kd)
   {
      centerOfMassHeightController.setProportionalGain(kp);
      centerOfMassHeightController.setDerivativeGain(kd);
   }

   public void initialize()
   {

   }

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
      centerOfMassTrajectoryGenerator.handlePelvisTrajectoryCommand(command);
   }

   public void handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      centerOfMassTrajectoryGenerator.handlePelvisHeightTrajectoryCommand(command);
   }

   public void goHome(double trajectoryTime)
   {
      centerOfMassTrajectoryGenerator.goHome(trajectoryTime);
   }

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

   public void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesToPack, boolean isInDoubleSupport)
   {
      centerOfMassTrajectoryGenerator.solve(coMHeightPartialDerivativesToPack, isInDoubleSupport);
   }

   // Temporary objects to reduce garbage collection.
   private final CoMHeightPartialDerivativesData coMHeightPartialDerivatives = new CoMHeightPartialDerivativesData();
   private final FramePoint comPosition = new FramePoint();
   private final FrameVector comVelocity = new FrameVector(worldFrame);
   private final FrameVector2d comXYVelocity = new FrameVector2d();
   private final FrameVector2d comXYAcceleration = new FrameVector2d();
   private final CoMHeightTimeDerivativesData comHeightDataBeforeSmoothing = new CoMHeightTimeDerivativesData();
   private final CoMHeightTimeDerivativesData comHeightDataAfterSmoothing = new CoMHeightTimeDerivativesData();
   private final CoMXYTimeDerivativesData comXYTimeDerivatives = new CoMXYTimeDerivativesData();
   private final FramePoint desiredCenterOfMassHeightPoint = new FramePoint(worldFrame);
   private final FramePoint pelvisPosition = new FramePoint();
   private final FramePoint2d comPositionAsFramePoint2d = new FramePoint2d();

   private final Twist currentPelvisTwist = new Twist();

   public double computeDesiredCoMHeightAcceleration(FrameVector2d desiredICPVelocity, boolean isInDoubleSupport, double omega0, boolean isRecoveringFromPush,
         FeetManager feetManager)
   {
      solve(coMHeightPartialDerivatives, isInDoubleSupport);

      comPosition.setToZero(centerOfMassFrame);
      centerOfMassJacobian.getCenterOfMassVelocity(comVelocity);
      comPosition.changeFrame(worldFrame);
      comVelocity.changeFrame(worldFrame);

      double zCurrent = comPosition.getZ();
      double zdCurrent = comVelocity.getZ();

      if (controlPelvisHeightInsteadOfCoMHeight.getBooleanValue())
      {
         pelvisPosition.setToZero(pelvisFrame);
         pelvisPosition.changeFrame(worldFrame);
         zCurrent = pelvisPosition.getZ();
         twistCalculator.getTwistOfBody(pelvis, currentPelvisTwist);
         currentPelvisTwist.changeFrame(worldFrame);
         zdCurrent = comVelocity.getZ(); // Just use com velocity for now for damping...
      }

      // TODO: use current omega0 instead of previous
      comXYVelocity.setIncludingFrame(comVelocity.getReferenceFrame(), comVelocity.getX(), comVelocity.getY());
      if (desiredICPVelocity.containsNaN())
      {
         System.err.println("Desired ICP velocity contains NaN");
         comXYAcceleration.setToZero(desiredICPVelocity.getReferenceFrame());
      }
      else
      {
         comXYAcceleration.setIncludingFrame(desiredICPVelocity);
      }
      comXYAcceleration.sub(comXYVelocity);
      comXYAcceleration.scale(omega0); // MathTools.square(omega0.getDoubleValue()) * (com.getX() - copX);

      // FrameVector2d comd2dSquared = new FrameVector2d(comXYVelocity.getReferenceFrame(), comXYVelocity.getX() * comXYVelocity.getX(), comXYVelocity.getY() * comXYVelocity.getY());
      comPosition.getFramePoint2d(comPositionAsFramePoint2d);
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
         feetManager.correctCoMHeight(desiredICPVelocity, zCurrent, comHeightDataAfterSmoothing);

      comHeightDataAfterSmoothing.getComHeight(desiredCenterOfMassHeightPoint);
      desiredCoMHeightCorrected.set(desiredCenterOfMassHeightPoint.getZ());
      desiredCoMHeightVelocityCorrected.set(comHeightDataAfterSmoothing.getComHeightVelocity());
      desiredCoMHeightAccelerationCorrected.set(comHeightDataAfterSmoothing.getComHeightAcceleration());

      comHeightDataAfterSmoothing.getComHeight(desiredCenterOfMassHeightPoint);

      double zDesired = desiredCenterOfMassHeightPoint.getZ();
      double zdDesired = comHeightDataAfterSmoothing.getComHeightVelocity();
      double zddFeedForward = comHeightDataAfterSmoothing.getComHeightAcceleration();

      double zddDesired = centerOfMassHeightController.compute(zCurrent, zDesired, zdCurrent, zdDesired) + zddFeedForward;

      // In a recovering context, accelerating upwards is just gonna make the robot fall faster. We should even consider letting the robot fall a bit.
      if (isRecoveringFromPush)
         zddDesired = Math.min(0.0, zddDesired);

      double epsilon = 1e-12;
      zddDesired = MathTools.clamp(zddDesired, -gravity + epsilon, Double.POSITIVE_INFINITY);

      return zddDesired;
   }
}
