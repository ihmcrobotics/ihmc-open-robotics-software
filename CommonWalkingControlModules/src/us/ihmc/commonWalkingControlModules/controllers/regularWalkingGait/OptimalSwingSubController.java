package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.BalanceOnOneLegConfiguration;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PreSwingControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlOnlyModule;
import us.ihmc.commonWalkingControlModules.controlModules.LegJointPositionControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.kinematics.BodyPositionInTimeEstimator;
import us.ihmc.commonWalkingControlModules.kinematics.InverseKinematicsException;
import us.ihmc.commonWalkingControlModules.kinematics.LegInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.optimalSwing.LegConfigurationData;
import us.ihmc.commonWalkingControlModules.optimalSwing.LegTorqueData;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Twist;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class OptimalSwingSubController implements SwingSubController
{
   private static final LegJointName[] legJointNames = new LegJointName[] { LegJointName.HIP_YAW, LegJointName.HIP_PITCH, LegJointName.HIP_ROLL,
         LegJointName.KNEE, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL };

   private final String name = getClass().getSimpleName();
   
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final CommonWalkingReferenceFrames referenceFrames;
   private final BodyPositionInTimeEstimator bodyPositionInTimeEstimator;
   private final CouplingRegistry couplingRegistry;
   private final LegInverseKinematicsCalculator inverseKinematicsCalculator;
   private final ProcessedSensorsInterface processedSensors;
   private final DesiredFootstepCalculator desiredFootstepCalculator;
   
   private final LegConfigurationData legConfigurationData;
   private final LegTorqueData legTorqueData;

   private final PreSwingControlModule preSwingControlModule;

   private final DoubleYoVariable swingDuration = new DoubleYoVariable("swingDuration", "The duration of the swing movement. [s]", registry);

   private final SideDependentList<YoFramePoint> desiredPositions = new SideDependentList<YoFramePoint>();
   private final SideDependentList<YoFrameOrientation> desiredOrientations = new SideDependentList<YoFrameOrientation>();

   
   private final SideDependentList<LegJointPositions> desiredJointAngles = new SideDependentList<LegJointPositions>();
   private final SideDependentList<LegJointVelocities> desiredJointVelocities = new SideDependentList<LegJointVelocities>();
   
   private final IntegerYoVariable numberOfInverseKinematicsErrors = new IntegerYoVariable("numberOfInverseKinematicsErrors", registry);

   private final DoubleYoVariable timeSpentInPreSwing = new DoubleYoVariable("timeSpentInPreSwing", "This is the time spent in Pre swing.", registry);
   private final DoubleYoVariable timeSpentInInitialSwing = new DoubleYoVariable("timeSpentInInitialSwing", "This is the time spent in initial swing.", registry);
   private final DoubleYoVariable timeSpentInMidSwing = new DoubleYoVariable("timeSpentInMidSwing", "This is the time spend in mid swing.", registry);
   private final DoubleYoVariable timeSpentInTerminalSwing = new DoubleYoVariable("timeSpentInTerminalSwing", "This is the time spent in terminal swing.", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", "This is the toal time spent in single support.", registry);

   private final DoubleYoVariable minimumTerminalSwingDuration = new DoubleYoVariable("minimumTerminalSwingDuration", "The minimum duration of terminal swing state. [s]", registry);
   private final DoubleYoVariable maximumTerminalSwingDuration = new DoubleYoVariable("maximumTerminalSwingDuration", "The maximum duration of terminal swing state. [s]", registry);

   private final DoubleYoVariable passiveHipCollapseTime = new DoubleYoVariable("passiveHipCollapseTime", registry);

   private final DoubleYoVariable estimatedSwingTimeRemaining = new DoubleYoVariable("estimatedSwingTimeRemaining", registry);


   private final BooleanYoVariable canGoToDoubleSupportFromLastTickState = new BooleanYoVariable("canGoToDoubleSupportFromLastTickState", registry);
   
   private final DoubleYoVariable positionErrorAtEndOfStepNorm = new DoubleYoVariable("positionErrorAtEndOfStepNorm", registry);
   private final DoubleYoVariable positionErrorAtEndOfStepX = new DoubleYoVariable("positionErrorAtEndOfStepX", registry);
   private final DoubleYoVariable positionErrorAtEndOfStepY = new DoubleYoVariable("positionErrorAtEndOfStepY", registry);
   private final SideDependentList<LegJointPositionControlModule> legJointPositionControlModules;
   private final SwingLegTorqueControlOnlyModule torqueControlModule;
   

   public OptimalSwingSubController(ProcessedSensorsInterface processedSensors, CommonWalkingReferenceFrames referenceFrames,
         DesiredFootstepCalculator desiredFootstepCalculator, CouplingRegistry couplingRegistry, LegInverseKinematicsCalculator inverseKinematicsCalculator,
         PreSwingControlModule preSwingControlModule,
         LegConfigurationData legConfigurationData, LegTorqueData legTorqueData,
         SwingLegTorqueControlOnlyModule swingLegTorqueControlModule,
         SideDependentList<LegJointPositionControlModule> legJointPositionControlModules,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      this.couplingRegistry = couplingRegistry;
      this.inverseKinematicsCalculator = inverseKinematicsCalculator;
      this.processedSensors = processedSensors;
      this.torqueControlModule = swingLegTorqueControlModule;

      this.preSwingControlModule = preSwingControlModule;
      this.legJointPositionControlModules = legJointPositionControlModules;

      this.legConfigurationData = legConfigurationData;
      this.legTorqueData = legTorqueData;
      
      bodyPositionInTimeEstimator = new BodyPositionInTimeEstimator(processedSensors, referenceFrames, couplingRegistry, registry);

      for (RobotSide side : RobotSide.values())
      {
         ReferenceFrame groundFrame = referenceFrames.getAnkleZUpFrame(side.getOppositeSide());
         desiredPositions.set(side, new YoFramePoint("finalDesiredPosition", side.getCamelCaseNameForMiddleOfExpression(), groundFrame, registry));
         desiredOrientations.set(side, new YoFrameOrientation("finalDesiredOrientation", side.getCamelCaseNameForMiddleOfExpression(), groundFrame, registry));
         
         desiredJointAngles.put(side, new LegJointPositions(side));
         desiredJointVelocities.put(side, new LegJointVelocities(legJointNames, side));
      }

      parentRegistry.addChild(registry);

      setParameters();
   }

   private void setParameters()
   {
      swingDuration.set(0.6);
      passiveHipCollapseTime.set(0.07);
      minimumTerminalSwingDuration.set(0.0);
      maximumTerminalSwingDuration.set(0.05);
      setEstimatedSwingTimeRemaining(swingDuration.getDoubleValue());
   }

   public void doPreSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(swingDuration.getDoubleValue());
      preSwingControlModule.doPreSwing(legTorquesToPackForSwingLeg, timeInState);
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
   
   private Transform3D computeDesiredTransform(ReferenceFrame pelvisFrame, FramePoint desiredFootPosition, Orientation desiredFootOrientation)
   {
      desiredFootOrientation.changeFrame(pelvisFrame);
      desiredFootPosition.changeFrame(pelvisFrame);
      Transform3D footToPelvis = createTransform(desiredFootOrientation, desiredFootPosition);

      return footToPelvis;
   }

   private static Transform3D createTransform(Orientation orientation, FramePoint framePoint)
   {
      orientation.checkReferenceFrameMatch(framePoint);
      Matrix3d rotationMatrix = orientation.getMatrix3d();
      Transform3D ret = new Transform3D(rotationMatrix, new Vector3d(framePoint.getPoint()), 1.0);

      return ret;
   }
   
   private void computeDesiredAnglesAtEndOfSwing(RobotSide swingSide, double swingTimeRemaining)
   {
      
      FramePoint desiredPosition = desiredPositions.get(swingSide).getFramePointCopy();
      Orientation desiredOrientation = desiredOrientations.get(swingSide).getFrameOrientationCopy();
      
      
      Transform3D footToPelvis = computeDesiredTransform(referenceFrames.getPelvisFrame(), desiredPosition, desiredOrientation);

      FramePose pelvisPoseInTime = bodyPositionInTimeEstimator.getPelvisPoseInTime(swingTimeRemaining, swingSide);
      Transform3D pelvisTransformInTime = new Transform3D();
      pelvisPoseInTime.getTransform3D(pelvisTransformInTime);
      pelvisTransformInTime.invert();
      
      footToPelvis.mul(pelvisTransformInTime, footToPelvis);
      
      
      try
      {
         inverseKinematicsCalculator.solve(desiredJointAngles.get(swingSide), footToPelvis, swingSide, desiredOrientation.getYawPitchRoll()[0]);
      } catch (InverseKinematicsException e)
      {
         numberOfInverseKinematicsErrors.increment();
      }
      
      for(LegJointName jointName : legJointNames)
      {
         desiredJointVelocities.get(swingSide).setJointVelocity(jointName, 0.0);
      }
   }
   

   private void doSwing(LegTorques legTorques, double timeInSwing)
   {
      RobotSide robotSide = legTorques.getRobotSide();
      
      double swingTimeRemaining = swingDuration.getDoubleValue() - timeInSwing;
      if(swingTimeRemaining < 0.0)
         swingTimeRemaining = 0.0;
      
      legConfigurationData.setSwingTimeRemaining(swingTimeRemaining);
      setEstimatedSwingTimeRemaining(swingTimeRemaining);
      
      
      Orientation upperBodyOrientation = processedSensors.getPelvisOrientationInFrame(ReferenceFrame.getWorldFrame());
      legConfigurationData.setUpperBodyOrientationInWorld(upperBodyOrientation);
      
      computeDesiredAnglesAtEndOfSwing(robotSide, 0.0);

      
      LegJointPositions legJointPositions = desiredJointAngles.get(robotSide);
      LegJointVelocities legJointVelocities = desiredJointVelocities.get(robotSide);
      legJointPositionControlModules.get(robotSide).packTorquesForLegJointsPositionControl(1.0, legTorques, legJointPositions, legJointVelocities);
      
      
      for(LegJointName jointName : legConfigurationData.getJointNames())
      {
         legConfigurationData.setCurrentJointAngle(jointName, processedSensors.getLegJointPosition(robotSide, jointName));
         legConfigurationData.setCurrentJointVelocity(jointName, processedSensors.getLegJointVelocity(robotSide, jointName));
         legConfigurationData.setFinalDesiredJointAngle(jointName, legJointPositions.getJointPosition(jointName));
      }
      
      Twist pelvisTwist = processedSensors.getTwistOfPelvisWithRespectToWorld();
      Vector3d pelvisAngularVelocity = pelvisTwist.getAngularPartCopy();
      
      legConfigurationData.setFinalDesiredJointVelocity(LegJointName.HIP_ROLL, legConfigurationData.getFinalDesiredJointAngle(LegJointName.HIP_ROLL) - pelvisAngularVelocity.y);
      legConfigurationData.setFinalDesiredJointVelocity(LegJointName.HIP_PITCH, legConfigurationData.getFinalDesiredJointAngle(LegJointName.HIP_PITCH) - pelvisAngularVelocity.x);
      
//      legConfigurationData.setFinalDesiredJointVelocity(LegJointName.HIP_ROLL, -pelvisAngularVelocity.y);
//      legConfigurationData.setFinalDesiredJointVelocity(LegJointName.HIP_PITCH, -pelvisAngularVelocity.x);
      
      
      for(LegJointName jointName : legTorqueData.getJointNames())
      {
         legTorques.setTorque(jointName, legTorqueData.getDesiredJointTorque(jointName));
      }
      
      

   }
   
   private void holdPosition(LegTorques legTorques)
   {
      RobotSide robotSide = legTorques.getRobotSide();
      LegJointPositions legJointPositions = desiredJointAngles.get(robotSide);
      LegJointVelocities legJointVelocities = desiredJointVelocities.get(robotSide);
      LegJointAccelerations legJointAccelerations = new LegJointAccelerations(legJointNames, robotSide);
      
      for(LegJointName jointName : legJointNames)
      {
         legJointAccelerations.setJointAcceleration(jointName, 0.0);
      }
      torqueControlModule.compute(legTorques, legJointPositions, legJointVelocities, legJointAccelerations);

   }

   public void doTerminalSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(0.0);

      holdPosition(legTorquesToPackForSwingLeg);

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

      return inStateLongEnough && isCoMPastSweetSpot;
   }

   public boolean isDoneWithMidSwing(RobotSide swingSide, double timeInState)
   {
      return timeSpentInInitialSwing.getDoubleValue() + timeInState > swingDuration.getDoubleValue();
   }

   public boolean isDoneWithTerminalSwing(RobotSide swingSide, double timeInState)
   {

      boolean minimumTerminalSwingTimePassed = (timeInState > minimumTerminalSwingDuration.getDoubleValue());
      boolean maximumTerminalSwingTimePassed = (timeInState > maximumTerminalSwingDuration.getDoubleValue());

      boolean capturePointInsideSwingFoot = isCapturePointInsideFoot(swingSide);
      boolean capturePointInsideSupportFoot = isCapturePointInsideFoot(swingSide.getOppositeSide());

      if (capturePointInsideSupportFoot) return false; // Don't go in double support if ICP is still in support foot.
      return (maximumTerminalSwingTimePassed);
   }

   public boolean isDoneWithSwingInAir(double timeInState)
   {
      return (timeInState > swingDuration.getDoubleValue()) && timeInState > 2.0;
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

      FramePose desiredFootstepPose = desiredFootstep.getFootstepPose();

      FramePoint endPoint = new FramePoint(desiredFootstepPose.getPosition());
      endPoint.changeFrame(desiredPositions.get(swingLeg).getReferenceFrame());
      Orientation endOrientation = new Orientation(desiredFootstepPose.getOrientation());
      endOrientation.changeFrame(desiredOrientations.get(swingLeg).getReferenceFrame());

      // Setup the orientation trajectory
      desiredPositions.get(swingLeg).set(endPoint);
      desiredOrientations.get(swingLeg).set(endOrientation);
      legConfigurationData.setRobotSide(swingLeg);
      legConfigurationData.setcurrentlyInSwing(true);


   }

   public void doTransitionIntoMidSwing(RobotSide swingSide)
   {
   }

   public void doTransitionIntoTerminalSwing(RobotSide swingSide)
   {
   }

   public void doTransitionIntoSwingInAir(RobotSide swingLeg, BalanceOnOneLegConfiguration currentConfiguration)
   {
      
      legConfigurationData.setRobotSide(swingLeg);
      legConfigurationData.setcurrentlyInSwing(true);
      
      FramePoint point = currentConfiguration.getDesiredSwingFootPosition();
      desiredPositions.get(swingLeg).set(point);
      desiredOrientations.get(swingLeg).setYawPitchRoll(0.0, 0.0, 0.0);

   }


   public void doTransitionOutOfPreSwing(RobotSide swingSide)
   {
   }

   public void doTransitionOutOfInitialSwing(RobotSide swingSide)
   {
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
      legConfigurationData.setcurrentlyInSwing(false);
   }

   public void doTransitionOutOfSwingInAir(RobotSide swingLeg)
   {
      legConfigurationData.setcurrentlyInSwing(false);
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

   public boolean isDoneWithSwingInAir(RobotSide swingSide, double timeInState)
   {
      return swingDuration.getDoubleValue() < timeInState;
   }

}
