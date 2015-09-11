package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import java.util.EnumMap;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.configurations.BalanceOnOneLegConfiguration;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlOnlyModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.kinematics.BodyPositionInTimeEstimator;
import us.ihmc.commonWalkingControlModules.kinematics.LegInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.SwingLegAnglesAtEndOfStepEstimator;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.trajectories.JointSpaceTrajectoryGenerator;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;


public class JointSpaceSwingSubController implements SwingSubController
{
   private static final LegJointName[] legJointNames = new LegJointName[] { LegJointName.HIP_YAW, LegJointName.HIP_PITCH, LegJointName.HIP_ROLL,
         LegJointName.KNEE, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL };

   private final YoVariableRegistry registry;
   private final FullHumanoidRobotModel fullRobotModel;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final BodyPositionInTimeEstimator bodyPositionInTimeEstimator;
   private final JointSpaceTrajectoryGenerator jointSpaceTrajectoryGenerator;
   private final CouplingRegistry couplingRegistry;
   private final ProcessedSensorsInterface processedSensors;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final DesiredFootstepCalculator desiredFootstepCalculator;

   private final EnumMap<LegJointName, DoubleYoVariable> legTorquesAtBeginningOfStep = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final YoMinimumJerkTrajectory gravityCompensationTrajectory;

   private final SideDependentList<LegJointPositions> jointPositions = new SideDependentList<LegJointPositions>();
   private final SideDependentList<LegJointVelocities> jointVelocities = new SideDependentList<LegJointVelocities>();
   private final SideDependentList<LegJointAccelerations> jointAccelerations = new SideDependentList<LegJointAccelerations>();

   private final SideDependentList<YoFramePoint> desiredPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameOrientation> desiredOrientations = new SideDependentList<YoFrameOrientation>();

   private final EnumMap<LegJointName, DoubleYoVariable> legJointSetpoints = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> legJointSetpointsd = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> legJointSetpointsdd = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);

   private final DoubleYoVariable timeSpentInPreSwing;
   private final DoubleYoVariable timeSpentInInitialSwing;
   private final DoubleYoVariable timeSpentInMidSwing;
   private final DoubleYoVariable timeSpentInTerminalSwing;

   private final DoubleYoVariable minimumTerminalSwingDuration;
   private final DoubleYoVariable maximumTerminalSwingDuration;

   private final DoubleYoVariable compensateGravityForSwingLegTime;

   private final IntegerYoVariable numberOfViaPointsDuringWalk;

   private final BooleanYoVariable canGoToDoubleSupportFromLastTickState;

   private final DoubleYoVariable positionErrorAtEndOfStepNorm;
   private final DoubleYoVariable positionErrorAtEndOfStepX;
   private final DoubleYoVariable positionErrorAtEndOfStepY;

   private final SwingLegTorqueControlOnlyModule torqueControlModule;

   private final BooleanYoVariable useBodyPositionEstimation;

   private final ProcessedOutputsInterface processedOutputs;

   private final YoFrameVector positionInSupportLegAnkleZUp;

   public JointSpaceSwingSubController(String name, ProcessedSensorsInterface processedSensors, ProcessedOutputsInterface processedOutputs,
         FullHumanoidRobotModel fullRobotModel, SideDependentList<FootSwitchInterface> footSwitches, CommonHumanoidReferenceFrames referenceFrames,
         DesiredFootstepCalculator desiredFootstepCalculator, CouplingRegistry couplingRegistry, LegInverseKinematicsCalculator inverseKinematicsCalculator,
         SwingLegTorqueControlOnlyModule swingLegTorqueControlModule, SwingLegAnglesAtEndOfStepEstimator swingLegAnglesAtEndOfStepEstimator,
         DesiredHeadingControlModule desiredHeadingControlModule, double controlDT, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      this.referenceFrames = referenceFrames;
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      this.couplingRegistry = couplingRegistry;
      this.processedSensors = processedSensors;
      this.footSwitches = new SideDependentList<FootSwitchInterface>(footSwitches);
      this.fullRobotModel = fullRobotModel;
      this.torqueControlModule = swingLegTorqueControlModule;
      this.processedOutputs = processedOutputs;

      this.gravityCompensationTrajectory = new YoMinimumJerkTrajectory("gravityCompensationTrajectory", registry);

      bodyPositionInTimeEstimator = new BodyPositionInTimeEstimator(processedSensors, referenceFrames, desiredHeadingControlModule, couplingRegistry, registry);

      jointSpaceTrajectoryGenerator = new JointSpaceTrajectoryGenerator("jointSpaceTrajectory", 2, referenceFrames, inverseKinematicsCalculator,
            processedSensors, controlDT, yoGraphicsListRegistry, bodyPositionInTimeEstimator, swingLegAnglesAtEndOfStepEstimator, registry);

      timeSpentInPreSwing = new DoubleYoVariable("timeSpentInPreSwing", "This is the time spent in Pre swing.", registry);
      timeSpentInInitialSwing = new DoubleYoVariable("timeSpentInInitialSwing", "This is the time spent in initial swing.", registry);
      timeSpentInMidSwing = new DoubleYoVariable("timeSpentInMidSwing", "This is the time spend in mid swing.", registry);
      timeSpentInTerminalSwing = new DoubleYoVariable("timeSpentInTerminalSwing", "This is the time spent in terminal swing.", registry);

      for (RobotSide side : RobotSide.values)
      {
         jointPositions.set(side, new LegJointPositions(side));
         jointVelocities.set(side, new LegJointVelocities(legJointNames, side));
         jointAccelerations.set(side, new LegJointAccelerations(legJointNames, side));

         desiredPositions.set(side, new YoFramePoint("finalDesiredPosition", side.getCamelCaseNameForMiddleOfExpression(), ReferenceFrame.getWorldFrame(),
               registry));
         desiredOrientations.set(side,
               new YoFrameOrientation("finalDesiredOrientation", side.getCamelCaseNameForMiddleOfExpression(), ReferenceFrame.getWorldFrame(), registry));
      }

      for (LegJointName jointName : legJointNames)
      {
         legTorquesAtBeginningOfStep.put(jointName,
               new DoubleYoVariable(jointName.getCamelCaseNameForStartOfExpression() + "TorqueAtBeginningOfStep", registry));

         legJointSetpoints.put(jointName, new DoubleYoVariable("q_" + jointName.getShortUnderBarName() + "_setpoint", registry));
         legJointSetpointsd.put(jointName, new DoubleYoVariable("qd_" + jointName.getShortUnderBarName() + "_setpoint", registry));
         legJointSetpointsdd.put(jointName, new DoubleYoVariable("qdd_" + jointName.getShortUnderBarName() + "_setpoint", registry));
      }

      compensateGravityForSwingLegTime = new DoubleYoVariable("compensateGravityForSwingLegTime", registry);
      minimumTerminalSwingDuration = new DoubleYoVariable("minimumTerminalSwingDuration", "The minimum duration of terminal swing state. [s]", registry);

      maximumTerminalSwingDuration = new DoubleYoVariable("maximumTerminalSwingDuration", "The maximum duration of terminal swing state. [s]", registry);
      numberOfViaPointsDuringWalk = new IntegerYoVariable("numberOfViaPointsDuringWalk", registry);

      canGoToDoubleSupportFromLastTickState = new BooleanYoVariable("canGoToDoubleSupportFromLastTickState", registry);

      positionErrorAtEndOfStepNorm = new DoubleYoVariable("positionErrorAtEndOfStepNorm", registry);
      positionErrorAtEndOfStepX = new DoubleYoVariable("positionErrorAtEndOfStepX", registry);
      positionErrorAtEndOfStepY = new DoubleYoVariable("positionErrorAtEndOfStepY", registry);

      useBodyPositionEstimation = new BooleanYoVariable("useBodyPositionEstimation", registry);
      useBodyPositionEstimation.set(true);

      positionInSupportLegAnkleZUp = new YoFrameVector("positionInSupportLegAnkleZUp", ReferenceFrame.getWorldFrame(), registry);

      parentRegistry.addChild(registry);
   }

   public void setUseBodyPositionEstimation(boolean val)
   {
      useBodyPositionEstimation.set(val);
   }

   public void setParametersForM2V2Walking()
   {
      compensateGravityForSwingLegTime.set(0.02);
      minimumTerminalSwingDuration.set(0.0);
      maximumTerminalSwingDuration.set(0.05);
      numberOfViaPointsDuringWalk.set(1);
   }

   public void setParametersForM2V2PushRecovery()
   {
      compensateGravityForSwingLegTime.set(0.02);
      minimumTerminalSwingDuration.set(0.0);
      maximumTerminalSwingDuration.set(0.05);
      numberOfViaPointsDuringWalk.set(0);
      setEstimatedSwingTimeRemaining(couplingRegistry.getSingleSupportDuration());
   }

   public void doPreSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(couplingRegistry.getSingleSupportDuration());

      gravityCompensationTrajectory.computeTrajectory(timeInState);
      double factor = gravityCompensationTrajectory.getPosition();

      torqueControlModule.computePreSwing(legTorquesToPackForSwingLeg);

      for (LegJointName legJointName : legJointNames)
      {
         double newTau = legTorquesToPackForSwingLeg.getTorque(legJointName);
         double oldTau = legTorquesAtBeginningOfStep.get(legJointName).getDoubleValue();

         double tau = (1.0 - factor) * oldTau + factor * newTau;

         legTorquesToPackForSwingLeg.setTorque(legJointName, tau);
      }

      couplingRegistry.getDesiredUpperBodyWrench().scale(factor);

      updateGroundClearance(legTorquesToPackForSwingLeg.getRobotSide());

      timeSpentInPreSwing.set(timeInState);
   }

   public void doInitialSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      updateDesiredPositions(legTorquesToPackForSwingLeg.getRobotSide());

      doSwing(legTorquesToPackForSwingLeg, timeInState, true);
      timeSpentInInitialSwing.set(timeInState);
   }

   public void doMidSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      updateDesiredPositions(legTorquesToPackForSwingLeg.getRobotSide());

      doSwing(legTorquesToPackForSwingLeg, timeSpentInInitialSwing.getDoubleValue() + timeInState, true);
      timeSpentInMidSwing.set(timeInState);
   }

   private void updateGroundClearance(RobotSide robotSide)
   {
      FramePoint swingFootPoint = new FramePoint(referenceFrames.getAnkleZUpFrame(robotSide));
      swingFootPoint.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide.getOppositeSide()));

      positionInSupportLegAnkleZUp.set(swingFootPoint.getVectorCopy());
   }

   private void doSwing(LegTorques legTorques, double timeInSwing, boolean useBodyPositionEstimation)
   {
      RobotSide swingLeg = legTorques.getRobotSide();
      updateGroundClearance(swingLeg);
      useBodyPositionEstimation = useBodyPositionEstimation & this.useBodyPositionEstimation.getBooleanValue();

      FramePoint desiredPosition = desiredPositions.get(swingLeg).getFramePointCopy();
      FrameOrientation desiredOrientation = desiredOrientations.get(swingLeg).getFrameOrientationCopy();

      LegJointPositions legJointPositions = jointPositions.get(swingLeg);
      LegJointVelocities legJointVelocities = jointVelocities.get(swingLeg);
      LegJointAccelerations legJointAccelerations = jointAccelerations.get(swingLeg);

      jointSpaceTrajectoryGenerator.updateEndPoint(desiredPosition, desiredOrientation, timeInSwing, useBodyPositionEstimation);
      jointSpaceTrajectoryGenerator.compute(legJointPositions, legJointVelocities, legJointAccelerations, timeInSwing);

      torqueControlModule.compute(legTorques, legJointPositions, legJointVelocities, legJointAccelerations);

      for (LegJointName jointName : legJointNames)
      {
         legJointSetpoints.get(jointName).set(legJointPositions.getJointPosition(jointName));
         legJointSetpointsd.get(jointName).set(legJointVelocities.getJointVelocity(jointName));
         legJointSetpointsdd.get(jointName).set(legJointAccelerations.getJointAcceleration(jointName));
      }

      setEstimatedSwingTimeRemaining(jointSpaceTrajectoryGenerator.getEstimatedTimeRemaining(timeInSwing));
   }

   public void doTerminalSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(0.0);
      updateDesiredPositions(legTorquesToPackForSwingLeg.getRobotSide());
      doSwing(legTorquesToPackForSwingLeg, jointSpaceTrajectoryGenerator.getSwingEndTime(), true);

      timeSpentInTerminalSwing.set(timeInState);

      canGoToDoubleSupportFromLastTickState.set(true);
   }

   public void doSwingInAir(LegTorques legTorques, double timeInState)
   {
      doSwing(legTorques, timeInState, false);
      canGoToDoubleSupportFromLastTickState.set(true);
   }

   public boolean isDoneWithPreSwingC(RobotSide loadingLeg, double timeInState)
   {
      return (timeInState > compensateGravityForSwingLegTime.getDoubleValue());
   }

   public boolean isDoneWithInitialSwing(RobotSide swingSide, double timeInState)
   {
      RobotSide oppositeSide = swingSide.getOppositeSide();
      ReferenceFrame stanceAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(oppositeSide);
      FramePoint comProjection = processedSensors.getCenterOfMassGroundProjectionInFrame(stanceAnkleZUpFrame);
      FramePoint2d sweetSpot = couplingRegistry.getOldBipedSupportPolygons().getSweetSpotCopy(oppositeSide);
      sweetSpot.changeFrame(stanceAnkleZUpFrame);
      boolean inStateLongEnough = timeInState > 0.05;
      boolean isCoMPastSweetSpot = comProjection.getX() > sweetSpot.getX();
      boolean trajectoryIsDone = jointSpaceTrajectoryGenerator.isDoneWithSwing(timeInState);

      return inStateLongEnough && (isCoMPastSweetSpot || trajectoryIsDone);
   }

   public boolean isDoneWithMidSwing(RobotSide swingSide, double timeInState)
   {
      return jointSpaceTrajectoryGenerator.isDoneWithSwing(timeSpentInInitialSwing.getDoubleValue() + timeInState);
   }

   public boolean isDoneWithTerminalSwing(RobotSide swingSide, double timeInState)
   {
      boolean footOnGround = footSwitches.get(swingSide).hasFootHitGround();

      boolean minimumTerminalSwingTimePassed = (timeInState > minimumTerminalSwingDuration.getDoubleValue());
      boolean capturePointInsideSupportFoot = isCapturePointInsideFoot(swingSide.getOppositeSide());

      if (capturePointInsideSupportFoot)
         return false; // Don't go in double support if ICP is still in support foot.

      return (footOnGround && minimumTerminalSwingTimePassed);
   }

   public boolean isDoneWithSwingInAir(RobotSide swingSide, double timeInState)
   {
      return jointSpaceTrajectoryGenerator.isDoneWithSwing(timeInState) && timeInState > 2.0;
   }

   public void doTransitionIntoPreSwing(RobotSide swingSide)
   {
      desiredFootstepCalculator.initializeDesiredFootstep(swingSide.getOppositeSide());

      // Reset the timers
      timeSpentInPreSwing.set(0.0);
      timeSpentInInitialSwing.set(0.0);
      timeSpentInMidSwing.set(0.0);
      timeSpentInTerminalSwing.set(0.0);
      canGoToDoubleSupportFromLastTickState.set(false);

      for (LegJointName jointName : legJointNames)
      {
         legTorquesAtBeginningOfStep.get(jointName).set(processedOutputs.getDesiredLegJointTorque(swingSide, jointName));
      }
      gravityCompensationTrajectory.setParams(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, compensateGravityForSwingLegTime.getDoubleValue());
   }

   private void updateDesiredPositions(RobotSide swingSide)
   {
      Footstep desiredFootstep = couplingRegistry.getDesiredFootstep();
      FramePose desiredFootstepPose = new FramePose();
      desiredFootstep.getPose(desiredFootstepPose);

      FramePoint endPoint = new FramePoint();
      desiredFootstepPose.getPositionIncludingFrame(endPoint);
      endPoint.changeFrame(desiredPositions.get(swingSide).getReferenceFrame());
      FrameOrientation endOrientation = new FrameOrientation();
      desiredFootstepPose.getOrientationIncludingFrame(endOrientation);
      endOrientation.changeFrame(desiredOrientations.get(swingSide).getReferenceFrame());

      // Setup the orientation trajectory
      desiredPositions.get(swingSide).set(endPoint);
      desiredOrientations.get(swingSide).set(endOrientation);
   }

   public void doTransitionIntoInitialSwing(RobotSide swingSide)
   {
      initializeToCurrentJointValues(swingSide);
      updateDesiredPositions(swingSide);
      jointSpaceTrajectoryGenerator.initialize(swingSide, jointPositions.get(swingSide), jointVelocities.get(swingSide), jointAccelerations.get(swingSide),
            desiredPositions.get(swingSide).getFramePointCopy(), desiredOrientations.get(swingSide).getFrameOrientationCopy(),
            couplingRegistry.getSingleSupportDuration(), numberOfViaPointsDuringWalk.getIntegerValue(), true);
      footSwitches.get(swingSide).reset();
   }

   public void doTransitionIntoMidSwing(RobotSide swingSide)
   {
   }

   public void doTransitionIntoTerminalSwing(RobotSide swingSide)
   {
      torqueControlModule.setAnkleGainsSoft(swingSide);
   }

   public void doTransitionIntoSwingInAir(RobotSide swingLeg, BalanceOnOneLegConfiguration currentConfiguration)
   {
      initializeToCurrentJointValues(swingLeg);
      FramePoint point = currentConfiguration.getDesiredSwingFootPosition();
      point.changeFrame(desiredPositions.get(swingLeg).getReferenceFrame());
      desiredPositions.get(swingLeg).set(point);
      desiredOrientations.get(swingLeg).setYawPitchRoll(0.0, 0.0, 0.0);

      jointSpaceTrajectoryGenerator.initialize(swingLeg, jointPositions.get(swingLeg), jointVelocities.get(swingLeg), jointAccelerations.get(swingLeg), point,
            desiredOrientations.get(swingLeg).getFrameOrientationCopy(), couplingRegistry.getSingleSupportDuration(), 0, false);
   }

   private void initializeToCurrentJointValues(RobotSide swingLeg)
   {
      LegJointPositions swingPositions = jointPositions.get(swingLeg);
      LegJointVelocities swingVelocities = jointVelocities.get(swingLeg);
      LegJointAccelerations swingAccelerations = jointAccelerations.get(swingLeg);

      for (LegJointName jointName : legJointNames)
      {
         OneDoFJoint legJoint = fullRobotModel.getLegJoint(swingLeg, jointName);
         swingPositions.setJointPosition(jointName, legJoint.getQ());
         swingVelocities.setJointVelocity(jointName, 0.0);//legJoint.getQd());
         swingAccelerations.setJointAcceleration(jointName, 0.0);//legJoint.getQdd());
      }
   }

   public void doTransitionOutOfPreSwing(RobotSide swingSide)
   {
   }

   public void doTransitionOutOfInitialSwing(RobotSide swingSide)
   {
      torqueControlModule.setAnkleGainsDefault(swingSide);
   }

   public void doTransitionOutOfMidSwing(RobotSide swingSide)
   {
   }

   public void doTransitionOutOfTerminalSwing(RobotSide swingSide)
   {
      FramePoint currentPosition = new FramePoint(referenceFrames.getAnkleZUpFrame(swingSide));
      FramePoint desiredPosition = desiredPositions.get(swingSide).getFramePointCopy();
      currentPosition.changeFrame(desiredPosition.getReferenceFrame());
      positionErrorAtEndOfStepNorm.set(desiredPosition.distance(currentPosition));
      currentPosition.sub(desiredPosition);
      positionErrorAtEndOfStepX.set(currentPosition.getX());
      positionErrorAtEndOfStepY.set(currentPosition.getY());
   }

   public void doTransitionOutOfSwingInAir(RobotSide swingLeg)
   {
      RobotSide supportLeg = swingLeg.getOppositeSide();
      desiredFootstepCalculator.initializeDesiredFootstep(supportLeg);
      // TODO: sort of nasty, but otherwise the swing trajectory won't be initialized correctly in doTransitionIntoInitialSwing:
      couplingRegistry.setDesiredFootstep(desiredFootstepCalculator.updateAndGetDesiredFootstep(swingLeg.getOppositeSide()));
   }

   public boolean canWeStopNow()
   {
      return true;
   }

   public boolean isReadyForDoubleSupport(RobotSide swingLeg)
   {
      FramePoint swingAnkle = new FramePoint(referenceFrames.getAnkleZUpFrame(swingLeg));
      swingAnkle.changeFrame(referenceFrames.getAnkleZUpFrame(swingLeg.getOppositeSide()));
      double deltaFootHeight = swingAnkle.getZ();
      double maxFootHeight = 0.02;

      return canGoToDoubleSupportFromLastTickState.getBooleanValue() && (deltaFootHeight < maxFootHeight);
   }

   public double getEstimatedSwingTimeRemaining()
   {
      return couplingRegistry.getEstimatedSwingTimeRemaining();
   }

   private void setEstimatedSwingTimeRemaining(double timeRemaining)
   {
      couplingRegistry.setEstimatedSwingTimeRemaining(timeRemaining);
   }

   private boolean isCapturePointInsideFoot(RobotSide swingSide)
   {
      FrameConvexPolygon2d footPolygon = couplingRegistry.getOldBipedSupportPolygons().getFootPolygonInAnkleZUp(swingSide);
      FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(footPolygon.getReferenceFrame()).toFramePoint2d();

      boolean capturePointInsideFoot = footPolygon.isPointInside(capturePoint);

      return capturePointInsideFoot;
   }

   public void initialize()
   {
      // TODO Auto-generated method stub
   }

   public void doPreSwingInAir(LegTorques legTorques, double timeInState)
   {
      doPreSwing(legTorques, timeInState);
   }

   public boolean isDoneWithPreSwingInAir(RobotSide swingSide, double timeInState)
   {
      return isDoneWithPreSwingC(swingSide, timeInState);
   }

   public void doTransitionIntoPreSwingInAir(RobotSide swingSide)
   {
      doTransitionIntoPreSwing(swingSide);
   }

   public void doTransitionOutOfPreSwingInAir(RobotSide swingLeg)
   {
      doTransitionOutOfPreSwing(swingLeg);
   }
}