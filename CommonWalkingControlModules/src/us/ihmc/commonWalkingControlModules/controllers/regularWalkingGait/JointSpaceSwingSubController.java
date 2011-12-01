package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import java.util.HashMap;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

import us.ihmc.commonWalkingControlModules.configurations.BalanceOnOneLegConfiguration;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PreSwingControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlOnlyModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.kinematics.BodyPositionInTimeEstimator;
import us.ihmc.commonWalkingControlModules.kinematics.LegInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.commonWalkingControlModules.trajectories.JointSpaceTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RevoluteJoint;

public class JointSpaceSwingSubController implements SwingSubController
{
   private static final LegJointName[] legJointNames = new LegJointName[] { LegJointName.HIP_YAW, LegJointName.HIP_PITCH, LegJointName.HIP_ROLL,
         LegJointName.KNEE, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL };

   private final YoVariableRegistry registry;
   private final FullRobotModel fullRobotModel;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final BodyPositionInTimeEstimator bodyPositionInTimeEstimator;
   private final JointSpaceTrajectoryGenerator jointSpaceTrajectoryGenerator;
   private final CouplingRegistry couplingRegistry;
   private final ProcessedSensorsInterface processedSensors;
   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final DesiredFootstepCalculator desiredFootstepCalculator;

   private final PreSwingControlModule preSwingControlModule;

   private final DoubleYoVariable swingDuration;

   private final SideDependentList<LegJointPositions> jointPositions = new SideDependentList<LegJointPositions>();
   private final SideDependentList<LegJointVelocities> jointVelocities = new SideDependentList<LegJointVelocities>();
   private final SideDependentList<LegJointAccelerations> jointAccelerations = new SideDependentList<LegJointAccelerations>();

   private final SideDependentList<YoFramePoint> desiredPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameOrientation> desiredOrientations = new SideDependentList<YoFrameOrientation>();

   private final HashMap<LegJointName, DoubleYoVariable> legJointSetpoints = new HashMap<LegJointName, DoubleYoVariable>();
   private final HashMap<LegJointName, DoubleYoVariable> legJointSetpointsd = new HashMap<LegJointName, DoubleYoVariable>();
   private final HashMap<LegJointName, DoubleYoVariable> legJointSetpointsdd = new HashMap<LegJointName, DoubleYoVariable>();

   private final DoubleYoVariable timeSpentInPreSwing;
   private final DoubleYoVariable timeSpentInInitialSwing;
   private final DoubleYoVariable timeSpentInMidSwing;
   private final DoubleYoVariable timeSpentInTerminalSwing;
   private final DoubleYoVariable singleSupportDuration;

   private final DoubleYoVariable minimumTerminalSwingDuration;
   private final DoubleYoVariable maximumTerminalSwingDuration;

   private final DoubleYoVariable passiveHipCollapseTime;

   private final DoubleYoVariable estimatedSwingTimeRemaining;

   private final IntegerYoVariable numberOfViaPointsDuringWalk;

   private final BooleanYoVariable canGoToDoubleSupportFromLastTickState;
   
   private final DoubleYoVariable positionErrorAtEndOfStepNorm;
   private final DoubleYoVariable positionErrorAtEndOfStepX;
   private final DoubleYoVariable positionErrorAtEndOfStepY;
   
   
   private final SwingLegTorqueControlOnlyModule torqueControlModule;

   public JointSpaceSwingSubController(String name, ProcessedSensorsInterface processedSensors, FullRobotModel fullRobotModel,
         SideDependentList<FootSwitchInterface> footSwitches, CommonWalkingReferenceFrames referenceFrames,
         DesiredFootstepCalculator desiredFootstepCalculator, CouplingRegistry couplingRegistry, LegInverseKinematicsCalculator inverseKinematicsCalculator,
         SwingLegTorqueControlOnlyModule swingLegTorqueControlModule, PreSwingControlModule preSwingControlModule, double controlDT,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      this.referenceFrames = referenceFrames;
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      this.couplingRegistry = couplingRegistry;
      this.swingDuration = new DoubleYoVariable("swingDuration", "The duration of the swing movement. [s]", registry);
      this.processedSensors = processedSensors;
      this.footSwitches = new SideDependentList<FootSwitchInterface>(footSwitches);
      this.fullRobotModel = fullRobotModel;
      this.torqueControlModule = swingLegTorqueControlModule;
      this.preSwingControlModule = preSwingControlModule;

      
      bodyPositionInTimeEstimator = new BodyPositionInTimeEstimator(processedSensors, referenceFrames, couplingRegistry, registry);
      jointSpaceTrajectoryGenerator = new JointSpaceTrajectoryGenerator("jointSpaceTrajectory", 1, referenceFrames, inverseKinematicsCalculator, processedSensors, controlDT,
            dynamicGraphicObjectsListRegistry, bodyPositionInTimeEstimator, registry);

      timeSpentInPreSwing = new DoubleYoVariable("timeSpentInPreSwing", "This is the time spent in Pre swing.", registry);
      timeSpentInInitialSwing = new DoubleYoVariable("timeSpentInInitialSwing", "This is the time spent in initial swing.", registry);
      timeSpentInMidSwing = new DoubleYoVariable("timeSpentInMidSwing", "This is the time spend in mid swing.", registry);
      timeSpentInTerminalSwing = new DoubleYoVariable("timeSpentInTerminalSwing", "This is the time spent in terminal swing.", registry);
      singleSupportDuration = new DoubleYoVariable("singleSupportDuration", "This is the toal time spent in single support.", registry);

      for (RobotSide side : RobotSide.values())
      {
         jointPositions.set(side, new LegJointPositions(side));
         jointVelocities.set(side, new LegJointVelocities(legJointNames, side));
         jointAccelerations.set(side, new LegJointAccelerations(legJointNames, side));

         ReferenceFrame groundFrame = referenceFrames.getAnkleZUpFrame(side.getOppositeSide());
         desiredPositions.set(side, new YoFramePoint("finalDesiredPosition", side.getCamelCaseNameForMiddleOfExpression(), groundFrame, registry));
         desiredOrientations.set(side, new YoFrameOrientation("finalDesiredOrientation", side.getCamelCaseNameForMiddleOfExpression(), groundFrame, registry));
      }

      for (LegJointName jointName : legJointNames)
      {
         legJointSetpoints.put(jointName, new DoubleYoVariable("q_" + jointName.getShortUnderBarName() + "_setpoint", registry));
         legJointSetpointsd.put(jointName, new DoubleYoVariable("qd_" + jointName.getShortUnderBarName() + "_setpoint", registry));
         legJointSetpointsdd.put(jointName, new DoubleYoVariable("qdd_" + jointName.getShortUnderBarName() + "_setpoint", registry));
      }

      parentRegistry.addChild(registry);

      estimatedSwingTimeRemaining = new DoubleYoVariable("estimatedSwingTimeRemaining", registry);

      passiveHipCollapseTime = new DoubleYoVariable("passiveHipCollapseTime", registry);
      minimumTerminalSwingDuration = new DoubleYoVariable("minimumTerminalSwingDuration", "The minimum duration of terminal swing state. [s]", registry);

      maximumTerminalSwingDuration = new DoubleYoVariable("maximumTerminalSwingDuration", "The maximum duration of terminal swing state. [s]", registry);
      numberOfViaPointsDuringWalk = new IntegerYoVariable("numberOfViaPointsDuringWalk", registry);

      canGoToDoubleSupportFromLastTickState = new BooleanYoVariable("canGoToDoubleSupportFromLastTickState", registry);
      
      positionErrorAtEndOfStepNorm = new DoubleYoVariable("positionErrorAtEndOfStepNorm", registry);
      positionErrorAtEndOfStepX = new DoubleYoVariable("positionErrorAtEndOfStepX", registry);
      positionErrorAtEndOfStepY = new DoubleYoVariable("positionErrorAtEndOfStepY", registry);

      setParameters();
   }

   private void setParameters()
   {
      swingDuration.set(0.55);
      passiveHipCollapseTime.set(0.07);
      minimumTerminalSwingDuration.set(0.0);
      maximumTerminalSwingDuration.set(0.05);
      numberOfViaPointsDuringWalk.set(1);
      setEstimatedSwingTimeRemaining(swingDuration.getDoubleValue());
   }

   public void doPreSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(swingDuration.getDoubleValue());
      preSwingControlModule.doPreSwing(legTorquesToPackForSwingLeg, timeInState);
      torqueControlModule.computePreSwing(legTorquesToPackForSwingLeg.getRobotSide());
      timeSpentInPreSwing.set(timeInState);
   }

   private void updateFinalDesiredPosition(RobotSide swingLeg)
   {
      FramePose desiredFootstepPose = couplingRegistry.getDesiredFootstep().getFootstepPose();

      FramePoint desiredSwingFootPosition = desiredFootstepPose.getPosition().changeFrameCopy(desiredPositions.get(swingLeg).getReferenceFrame());
      Orientation desiredSwingFootOrientation = desiredFootstepPose.getOrientation().changeFrameCopy(desiredOrientations.get(swingLeg).getReferenceFrame());

      desiredPositions.get(swingLeg).set(desiredSwingFootPosition);
      desiredOrientations.get(swingLeg).set(desiredSwingFootOrientation);

   }

   public void doInitialSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
//      updateFinalDesiredPosition(legTorquesToPackForSwingLeg.getRobotSide());
      doSwing(legTorquesToPackForSwingLeg, timeInState);
      timeSpentInInitialSwing.set(timeInState);
   }

   public void doMidSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
//      updateFinalDesiredPosition(legTorquesToPackForSwingLeg.getRobotSide());
      doSwing(legTorquesToPackForSwingLeg, timeSpentInInitialSwing.getDoubleValue() + timeInState);
      timeSpentInMidSwing.set(timeInState);
   }

   private void doSwing(LegTorques legTorques, double timeInSwing)
   {
      RobotSide swingLeg = legTorques.getRobotSide();

      FramePoint desiredPosition = desiredPositions.get(swingLeg).getFramePointCopy();
      Orientation desiredOrientation = desiredOrientations.get(swingLeg).getFrameOrientationCopy();

      LegJointPositions legJointPositions = jointPositions.get(swingLeg);
      LegJointVelocities legJointVelocities = jointVelocities.get(swingLeg);
      LegJointAccelerations legJointAccelerations = jointAccelerations.get(swingLeg);

      jointSpaceTrajectoryGenerator.updateEndPoint(desiredPosition, desiredOrientation, timeInSwing);
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

      doSwing(legTorquesToPackForSwingLeg, jointSpaceTrajectoryGenerator.getSwingEndTime());

      timeSpentInTerminalSwing.set(timeInState);
      
      canGoToDoubleSupportFromLastTickState.set(true);
   }

   public void doSwingInAir(LegTorques legTorques, double timeInState)
   {
      doSwing(legTorques, timeInState);
      canGoToDoubleSupportFromLastTickState.set(true);
   }

   public boolean isDoneWithPreSwingC(RobotSide loadingLeg, double timeInState)
   {
      return (timeInState > passiveHipCollapseTime.getDoubleValue());
   }

   public boolean isDoneWithInitialSwing(RobotSide swingSide, double timeInState)
   {
      RobotSide oppositeSide = swingSide.getOppositeSide();
      ReferenceFrame stanceAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(oppositeSide);
      FramePoint comProjection = processedSensors.getCenterOfMassGroundProjectionInFrame(stanceAnkleZUpFrame);
      FramePoint2d sweetSpot = couplingRegistry.getBipedSupportPolygons().getSweetSpotCopy(oppositeSide);
      sweetSpot.changeFrame(stanceAnkleZUpFrame);
      boolean inStateLongEnough = timeInState > 0.05;
      boolean isCoMPastSweetSpot = comProjection.getX() > sweetSpot.getX();
      boolean trajectoryIsDone = jointSpaceTrajectoryGenerator.isDoneWithSwing(timeInState);
      boolean footHitEarly = footSwitches.get(swingSide).hasFootHitGround();

      return inStateLongEnough && (isCoMPastSweetSpot || trajectoryIsDone || footHitEarly);
   }

   public boolean isDoneWithMidSwing(RobotSide swingSide, double timeInState)
   {
      return jointSpaceTrajectoryGenerator.isDoneWithSwing(timeSpentInInitialSwing.getDoubleValue() + timeInState);
   }

   public boolean isDoneWithTerminalSwing(RobotSide swingSide, double timeInState)
   {
      boolean footOnGround = footSwitches.get(swingSide).hasFootHitGround();

      boolean minimumTerminalSwingTimePassed = (timeInState > minimumTerminalSwingDuration.getDoubleValue());
      boolean maximumTerminalSwingTimePassed = (timeInState > maximumTerminalSwingDuration.getDoubleValue());

      boolean capturePointInsideSwingFoot = isCapturePointInsideFoot(swingSide);
      boolean capturePointInsideSupportFoot = isCapturePointInsideFoot(swingSide.getOppositeSide());

      if (capturePointInsideSupportFoot) return false; // Don't go in double support if ICP is still in support foot.
//      return ((footOnGround && minimumTerminalSwingTimePassed) || maximumTerminalSwingTimePassed || (capturePointInsideFoot && minimumTerminalSwingTimePassed));
      return (maximumTerminalSwingTimePassed);
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
      singleSupportDuration.set(0.0);
      canGoToDoubleSupportFromLastTickState.set(false);

   }

   public void doTransitionIntoInitialSwing(RobotSide swingLeg)
   {
      Footstep desiredFootstep = couplingRegistry.getDesiredFootstep();

      initializeToCurrentJointValues(swingLeg);
      FramePose desiredFootstepPose = desiredFootstep.getFootstepPose();

      FramePoint endPoint = new FramePoint(desiredFootstepPose.getPosition());
      endPoint.changeFrame(desiredPositions.get(swingLeg).getReferenceFrame());
      Orientation endOrientation = new Orientation(desiredFootstepPose.getOrientation());
      endOrientation.changeFrame(desiredOrientations.get(swingLeg).getReferenceFrame());

      // Setup the orientation trajectory
      desiredPositions.get(swingLeg).set(endPoint);
      desiredOrientations.get(swingLeg).set(endOrientation);

      jointSpaceTrajectoryGenerator.initialize(swingLeg, jointPositions.get(swingLeg), jointVelocities.get(swingLeg), jointAccelerations.get(swingLeg),
            endPoint, endOrientation, swingDuration.getDoubleValue(), numberOfViaPointsDuringWalk.getIntegerValue());
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
      desiredPositions.get(swingLeg).set(point);
      desiredOrientations.get(swingLeg).setYawPitchRoll(0.0, 0.0, 0.0);

      jointSpaceTrajectoryGenerator.initialize(swingLeg, jointPositions.get(swingLeg), jointVelocities.get(swingLeg), jointAccelerations.get(swingLeg), point,
            desiredOrientations.get(swingLeg).getFrameOrientationCopy(), swingDuration.getDoubleValue(), 0);
   }

   private void initializeToCurrentJointValues(RobotSide swingLeg)
   {
      LegJointPositions swingPositions = jointPositions.get(swingLeg);
      LegJointVelocities swingVelocities = jointVelocities.get(swingLeg);
      LegJointAccelerations swingAccelerations = jointAccelerations.get(swingLeg);

      for (LegJointName jointName : legJointNames)
      {
         RevoluteJoint legJoint = fullRobotModel.getLegJoint(swingLeg, jointName);
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
      currentPosition.changeFrame(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
      FramePoint desiredPosition = desiredPositions.get(swingSide).getFramePointCopy();
      positionErrorAtEndOfStepNorm.set(desiredPosition.distance(currentPosition));
      currentPosition.sub(desiredPosition);
      positionErrorAtEndOfStepX.set(currentPosition.getX());
      positionErrorAtEndOfStepY.set(currentPosition.getY());
      
      
      singleSupportDuration.set(timeSpentInPreSwing.getDoubleValue() + timeSpentInInitialSwing.getDoubleValue() + timeSpentInMidSwing.getDoubleValue()
            + timeSpentInTerminalSwing.getDoubleValue());
      couplingRegistry.setSingleSupportDuration(singleSupportDuration.getDoubleValue());
   }

   public void doTransitionOutOfSwingInAir(RobotSide swingLeg)
   {
   }

   public boolean canWeStopNow()
   {
      return true;
   }

   public boolean isReadyForDoubleSupport()
   {
      FramePoint swingAnkle = new FramePoint(referenceFrames.getAnkleZUpFrame(RobotSide.LEFT));
      swingAnkle.changeFrame(referenceFrames.getAnkleZUpFrame(RobotSide.RIGHT));
      double deltaFootHeight = Math.abs(swingAnkle.getZ());
      double maxFootHeight = 0.02;

      return canGoToDoubleSupportFromLastTickState.getBooleanValue() && (deltaFootHeight < maxFootHeight);

   }

   public double getEstimatedSwingTimeRemaining()
   {
      return estimatedSwingTimeRemaining.getDoubleValue();
   }

   private void setEstimatedSwingTimeRemaining(double timeRemaining)
   {
      estimatedSwingTimeRemaining.set(timeRemaining);
      couplingRegistry.setEstimatedSwingTimeRemaining(timeRemaining);
   }

   private boolean isCapturePointInsideFoot(RobotSide swingSide)
   {
      FrameConvexPolygon2d footPolygon = couplingRegistry.getBipedSupportPolygons().getFootPolygonInAnkleZUp(swingSide);
      FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(footPolygon.getReferenceFrame()).toFramePoint2d();

      boolean capturePointInsideFoot = footPolygon.isPointInside(capturePoint);

      return capturePointInsideFoot;
   }

   public void initialize()
   {
      // TODO Auto-generated method stub

   }

}
