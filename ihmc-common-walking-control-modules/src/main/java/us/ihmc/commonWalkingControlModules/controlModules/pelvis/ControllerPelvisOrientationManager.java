package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.PelvisOffsetWhileWalkingParameters;
import us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith.PelvisLeapOfFaithModule;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ControllerPelvisOrientationManager implements PelvisOrientationControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FrameQuaternion desiredPelvisOrientation = new FrameQuaternion();
   private final FrameVector3D desiredPelvisAngularVelocity = new FrameVector3D();
   private final FrameVector3D desiredPelvisAngularAcceleration = new FrameVector3D();
   private final FrameQuaternion desiredPelvisOrientationWithOffset = new FrameQuaternion();

   private final FrameQuaternion initialPelvisOrientation = new FrameQuaternion();
   private final FrameQuaternion finalPelvisOrientation = new FrameQuaternion();

   private final SimpleOrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;
   private final SimpleOrientationTrajectoryGenerator pelvisOrientationOffsetTrajectoryGenerator;
   private final PelvisLeapOfFaithModule leapOfFaithModule;

   private final YoDouble initialPelvisOrientationTime = new YoDouble("initialPelvisOrientationTime", registry);
   private final YoDouble initialPelvisOrientationOffsetTime = new YoDouble("initialPelvisOrientationOffsetTime", registry);
   private final YoDouble yoTime;

   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private Vector3DReadOnly pelvisAngularWeight = null;
   private final Vector3D tempWeight = new Vector3D();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

   private final FrameQuaternion tempOrientation = new FrameQuaternion();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();
   private final FrameVector3D tempAngularAcceleration = new FrameVector3D();

   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;
   private final ReferenceFrame midFeetZUpGroundFrame;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame desiredPelvisFrame;

   private final PID3DGainsReadOnly gains;

   private Footstep nextFootstep;
   private final ReferenceFrame nextSoleZUpFrame;
   private final ReferenceFrame nextSoleFrame;

   private final PelvisOffsetTrajectoryWhileWalking offsetTrajectoryWhileWalking;

   private final BooleanProvider useManualRotations = new BooleanParameter("UseManualPelvisRotation", registry, false);
   private final DoubleProvider desiredYawOffset = new DoubleParameter("UserDesiredYawOffset", registry, 0.0);
   private final DoubleProvider desiredPitchOffset = new DoubleParameter("UserDesiredPitchOffset", registry, 0.0);
   private final DoubleProvider desiredRollOffset = new DoubleParameter("UserDesiredRollOffset", registry, 0.0);

   public ControllerPelvisOrientationManager(PID3DGainsReadOnly gains, PelvisOffsetWhileWalkingParameters pelvisOffsetWhileWalkingParameters,
                                             LeapOfFaithParameters leapOfFaithParameters, HighLevelHumanoidControllerToolbox controllerToolbox,
                                             YoVariableRegistry parentRegistry)
   {
      yoTime = controllerToolbox.getYoTime();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      midFeetZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      soleZUpFrames = referenceFrames.getSoleZUpFrames();
      pelvisFrame = referenceFrames.getPelvisFrame();

      pelvisOrientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("pelvis", true, worldFrame, registry);

      this.gains = gains;
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      orientationFeedbackControlCommand.set(elevator, pelvis);
      selectionMatrix.resetSelection();

      desiredPelvisFrame = new ReferenceFrame("desiredPelvisFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            pelvisFrame.getTransformToDesiredFrame(transformToParent, getParent());
            transformToParent.setRotation(desiredPelvisOrientation);
         }
      };

      offsetTrajectoryWhileWalking = new PelvisOffsetTrajectoryWhileWalking(controllerToolbox, pelvisOffsetWhileWalkingParameters, registry);

      nextSoleFrame = new ReferenceFrame("nextSoleFrame", worldFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            nextFootstep.getSoleReferenceFrame().getTransformToDesiredFrame(transformToParent, getParent());
         }
      };
      nextSoleZUpFrame = new ZUpFrame(worldFrame, nextSoleFrame, "nextAnkleZUp");

      pelvisOrientationOffsetTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("pelvisOffset", false, desiredPelvisFrame, registry);
      leapOfFaithModule = new PelvisLeapOfFaithModule(soleZUpFrames, leapOfFaithParameters, registry);

      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3DReadOnly pelvisAngularWeight)
   {
      this.pelvisAngularWeight = pelvisAngularWeight;
   }

   public void setTrajectoryTime(double trajectoryTime)
   {
      if (trajectoryTime < 0.0)
      {
         throw new RuntimeException("Negative trajectory time: " + trajectoryTime);
      }
      pelvisOrientationTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
   }

   private void initialize(ReferenceFrame desiredTrajectoryFrame)
   {
      initializeTrajectoryFrame(desiredTrajectoryFrame);
      initializeTiming();
   }

   private void initializeTrajectoryFrame(ReferenceFrame desiredTrajectoryFrame)
   {
      initialPelvisOrientation.changeFrame(desiredTrajectoryFrame);
      finalPelvisOrientation.changeFrame(desiredTrajectoryFrame);

      pelvisOrientationTrajectoryGenerator.setReferenceFrame(desiredTrajectoryFrame);
      pelvisOrientationTrajectoryGenerator.setInitialOrientation(initialPelvisOrientation);
      pelvisOrientationTrajectoryGenerator.setFinalOrientation(finalPelvisOrientation);
      pelvisOrientationTrajectoryGenerator.initialize();

      desiredPelvisOrientation.setIncludingFrame(initialPelvisOrientation);
      desiredPelvisOrientation.changeFrame(worldFrame);
      desiredPelvisFrame.update();
   }

   private void initializeTiming()
   {
      initialPelvisOrientationTime.set(yoTime.getDoubleValue());
   }

   @Override
   public void doAction(double timeInState)
   {
      double deltaTime = yoTime.getDoubleValue() - initialPelvisOrientationTime.getDoubleValue();
      pelvisOrientationTrajectoryGenerator.compute(deltaTime);
      pelvisOrientationTrajectoryGenerator.getAngularData(desiredPelvisOrientation, desiredPelvisAngularVelocity, desiredPelvisAngularAcceleration);

      desiredPelvisOrientation.changeFrame(worldFrame);
      desiredPelvisAngularVelocity.changeFrame(worldFrame);
      desiredPelvisAngularAcceleration.changeFrame(worldFrame);
      desiredPelvisFrame.update();

      double deltaTimeOffset = yoTime.getDoubleValue() - initialPelvisOrientationOffsetTime.getDoubleValue();
      pelvisOrientationOffsetTrajectoryGenerator.compute(deltaTimeOffset);
      pelvisOrientationOffsetTrajectoryGenerator.getAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      if (useManualRotations.getValue())
      {
         tempOrientation.prependRollRotation(desiredRollOffset.getValue());
         tempOrientation.prependPitchRotation(desiredPitchOffset.getValue());
         tempOrientation.prependYawRotation(desiredYawOffset.getValue());
      }

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      offsetTrajectoryWhileWalking.update();
      offsetTrajectoryWhileWalking.addAngularOffset(tempOrientation);

      tempWeight.set(pelvisAngularWeight);
      leapOfFaithModule.update(deltaTime);
      leapOfFaithModule.updateAngularOffsets();
      leapOfFaithModule.addAngularOffset(tempOrientation);
      leapOfFaithModule.relaxAngularWeight(tempWeight);

      desiredPelvisOrientationWithOffset.setIncludingFrame(tempOrientation);
      desiredPelvisAngularVelocity.add(tempAngularVelocity);
      desiredPelvisAngularAcceleration.add(tempAngularAcceleration);

      orientationFeedbackControlCommand.setInverseDynamics(desiredPelvisOrientationWithOffset, desiredPelvisAngularVelocity, desiredPelvisAngularAcceleration);
      orientationFeedbackControlCommand.setWeightsForSolver(tempWeight);
      orientationFeedbackControlCommand.setGains(gains);
      orientationFeedbackControlCommand.setSelectionMatrix(selectionMatrix);
   }

   @Override
   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());

      pelvisOrientationOffsetTrajectoryGenerator.getOrientation(tempOrientation);

      tempOrientation.changeFrame(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      tempOrientation.setToZero(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void goToHomeFromOffset(FrameQuaternionReadOnly offset, double trajectoryTime)
   {
      initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());

      tempOrientation.setIncludingFrame(offset);
      tempOrientation.changeFrame(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      tempOrientation.setToZero(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void setOffset(FrameQuaternionReadOnly offset)
   {
      tempOrientation.setIncludingFrame(offset);
      tempOrientation.changeFrame(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(0.0);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void resetOrientationOffset()
   {
      tempOrientation.setToZero(desiredPelvisFrame);
      pelvisOrientationOffsetTrajectoryGenerator.setInitialOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setFinalOrientation(tempOrientation);
      pelvisOrientationOffsetTrajectoryGenerator.setTrajectoryTime(0.0);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
   }

   public void setToHoldCurrentInWorldFrame()
   {
      setToHoldCurrent(worldFrame);
   }

   public void setToHoldCurrent(ReferenceFrame trajectoryFrame)
   {
      tempOrientation.setToZero(pelvisFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.setIncludingFrame(tempOrientation);
      finalPelvisOrientation.setIncludingFrame(tempOrientation);
      desiredPelvisOrientation.setIncludingFrame(tempOrientation);

      resetOrientationOffset();
      initialize(trajectoryFrame);
   }

   public void centerInMidFeetZUpFrame(double trajectoryTime)
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);
      finalPelvisOrientation.setToZero(midFeetZUpGroundFrame);
      setTrajectoryTime(trajectoryTime);
      initialize(midFeetZUpGroundFrame);
   }

   public void setToHoldCurrentDesiredInMidFeetZUpFrame()
   {
      setToHoldCurrentDesired(midFeetZUpGroundFrame);
   }

   public void setToHoldCurrentDesiredInSupportFoot(RobotSide supportSide)
   {
      setToHoldCurrentDesired(soleZUpFrames.get(supportSide));
   }

   public void setToHoldCurrentDesired(ReferenceFrame desiredTrajectoryFrame)
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);
      finalPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);
      initialize(desiredTrajectoryFrame);
   }

   /** Go instantly to zero, no smooth interpolation. */
   public void setToZeroInMidFeetZUpFrame()
   {
      tempOrientation.setToZero(midFeetZUpGroundFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.setIncludingFrame(tempOrientation);
      finalPelvisOrientation.setIncludingFrame(tempOrientation);
      desiredPelvisOrientation.setIncludingFrame(tempOrientation);

      initialize(midFeetZUpGroundFrame);
   }

   public void moveToAverageInSupportFoot(RobotSide supportSide)
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);

      ReferenceFrame otherAnkleZUpFrame = soleZUpFrames.get(supportSide.getOppositeSide());
      ReferenceFrame supportAnkleZUpFrame = soleZUpFrames.get(supportSide);

      tempOrientation.setToZero(otherAnkleZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawOtherFoot = tempOrientation.getYaw();

      tempOrientation.setToZero(supportAnkleZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawSupportFoot = tempOrientation.getYaw();

      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(yawOtherFoot, yawSupportFoot);
      finalPelvisOrientation.setYawPitchRollIncludingFrame(worldFrame, finalDesiredPelvisYawAngle, 0.0, 0.0);

      initialize(supportAnkleZUpFrame);
   }

   public void setUpcomingFootstep(Footstep upcomingFootstep)
   {
      nextFootstep = upcomingFootstep;

      nextSoleFrame.update();
      nextSoleZUpFrame.update();

      offsetTrajectoryWhileWalking.setUpcomingFootstep(upcomingFootstep);
      leapOfFaithModule.setUpcomingFootstep(upcomingFootstep);
   }

   public void setTrajectoryFromFootstep()
   {
      updateTrajectoryFromFootstep();

      initializeTiming();
   }

   public void updateTrajectoryFromFootstep()
   {
      initialPelvisOrientation.setIncludingFrame(desiredPelvisOrientation);

      RobotSide upcomingFootstepSide = nextFootstep.getRobotSide();
      ReferenceFrame supportSoleFrame = soleZUpFrames.get(upcomingFootstepSide.getOppositeSide());

      nextFootstep.getOrientation(tempOrientation);
      tempOrientation.changeFrame(worldFrame);
      double yawFootstep = tempOrientation.getYaw();

      tempOrientation.setToZero(supportSoleFrame);
      tempOrientation.changeFrame(worldFrame);
      double yawSupportFoot = tempOrientation.getYaw();

      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(yawFootstep, yawSupportFoot);
      finalPelvisOrientation.setYawPitchRollIncludingFrame(worldFrame, finalDesiredPelvisYawAngle, 0.0, 0.0);

      initializeTrajectoryFrame(worldFrame);
   }

   public void initializeStanding()
   {
      setToHoldCurrentDesiredInMidFeetZUpFrame();
      offsetTrajectoryWhileWalking.initializeStanding();
      leapOfFaithModule.initializeStanding();
   }

   public void initializeTransfer(RobotSide transferToSide, double transferDuration, double swingDuration)
   {
      initializeTiming();
      offsetTrajectoryWhileWalking.initializeTransfer(transferToSide, transferDuration, swingDuration);
      leapOfFaithModule.initializeTransfer(transferDuration);
   }

   public void initializeSwing(RobotSide supportSide, double swingDuration, double nextTransferDuration, double nextSwingDuration)
   {
      initializeTiming();
      offsetTrajectoryWhileWalking.initializeSwing(supportSide, swingDuration, nextTransferDuration, nextSwingDuration);
      leapOfFaithModule.initializeSwing(swingDuration);
   }

   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return orientationFeedbackControlCommand;
   }
}