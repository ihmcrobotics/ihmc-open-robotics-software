package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import java.util.EnumMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.BalanceOnOneLegConfiguration;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlOnlyModule;
import us.ihmc.commonWalkingControlModules.controlModules.LegJointPositionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.swingLegTorqueControl.CraigPage300SwingLegTorqueControlOnlyModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.kinematics.BodyPositionInTimeEstimator;
import us.ihmc.commonWalkingControlModules.kinematics.DesiredJointVelocityCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.InverseKinematicsException;
import us.ihmc.commonWalkingControlModules.kinematics.LegInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.optimalSwing.LegConfigurationData;
import us.ihmc.commonWalkingControlModules.optimalSwing.LegTorqueData;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
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
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.trajectory.YoMinimumJerkTrajectory;

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
   private final ProcessedOutputsInterface processedOutputs;
   private final DesiredFootstepCalculator desiredFootstepCalculator;
   private final double controlDT;
   
   private final LegConfigurationData legConfigurationData;
   private final LegTorqueData legTorqueData;

   private final EnumMap<LegJointName, DoubleYoVariable> legTorquesAtBeginningOfStep = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final YoMinimumJerkTrajectory gravityCompensationTrajectory = new YoMinimumJerkTrajectory("gravityCompensationTrajectory", registry);

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

   private final DoubleYoVariable minimumTerminalSwingDuration = new DoubleYoVariable("minimumTerminalSwingDuration", "The minimum duration of terminal swing state. [s]", registry);
   private final DoubleYoVariable maximumTerminalSwingDuration = new DoubleYoVariable("maximumTerminalSwingDuration", "The maximum duration of terminal swing state. [s]", registry);

   private final DoubleYoVariable compensateGravityForSwingLegTime = new DoubleYoVariable("compensateGravityForSwingLegTime", registry);

   private final DoubleYoVariable estimatedSwingTimeRemaining = new DoubleYoVariable("estimatedSwingTimeRemaining", registry);


   private final BooleanYoVariable canGoToDoubleSupportFromLastTickState = new BooleanYoVariable("canGoToDoubleSupportFromLastTickState", registry);
   
   private final DoubleYoVariable positionErrorAtEndOfStepNorm = new DoubleYoVariable("positionErrorAtEndOfStepNorm", registry);
   private final DoubleYoVariable positionErrorAtEndOfStepX = new DoubleYoVariable("positionErrorAtEndOfStepX", registry);
   private final DoubleYoVariable positionErrorAtEndOfStepY = new DoubleYoVariable("positionErrorAtEndOfStepY", registry);
   private final SwingLegTorqueControlOnlyModule torqueControlModule;
   private final  SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators;
   
   private final DoubleYoVariable ikAlpha = new DoubleYoVariable("ikAlpha", registry);
   
//   private final EnumMap<LegJointName, AlphaFilteredYoVariable> filteredJointTorques = new EnumMap<LegJointName, AlphaFilteredYoVariable>(LegJointName.class);
   
   private final DoubleYoVariable jointVelocityBreakFrequency = new DoubleYoVariable("jointVelocityBreakFrequency", registry);
   private final EnumMap<LegJointName, AlphaFilteredYoVariable> filteredJointVelocities = new EnumMap<LegJointName, AlphaFilteredYoVariable>(LegJointName.class);
   
   private final SideDependentList<FootSwitchInterface> footSwitches;



   public OptimalSwingSubController(ProcessedSensorsInterface processedSensors, ProcessedOutputsInterface processedOutputs, CommonWalkingReferenceFrames referenceFrames,
         DesiredFootstepCalculator desiredFootstepCalculator, SideDependentList<FootSwitchInterface> footSwitches, CouplingRegistry couplingRegistry,
         SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators, LegInverseKinematicsCalculator inverseKinematicsCalculator,
         LegConfigurationData legConfigurationData, LegTorqueData legTorqueData, SwingLegTorqueControlOnlyModule swingLegTorqueControlModule,
         SideDependentList<LegJointPositionControlModule> legJointPositionControlModules, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         double controlDT, YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      this.couplingRegistry = couplingRegistry;
      this.inverseKinematicsCalculator = inverseKinematicsCalculator;
      this.processedSensors = processedSensors;
      this.processedOutputs = processedOutputs;
      this.torqueControlModule = swingLegTorqueControlModule;
      this.controlDT = controlDT;
      this.desiredJointVelocityCalculators = desiredJointVelocityCalculators;

      this.legConfigurationData = legConfigurationData;
      this.legTorqueData = legTorqueData;
      
      this.footSwitches = footSwitches;

      bodyPositionInTimeEstimator = new BodyPositionInTimeEstimator(processedSensors, referenceFrames, couplingRegistry, registry);

      for (RobotSide side : RobotSide.values())
      {
         ReferenceFrame groundFrame = referenceFrames.getAnkleZUpFrame(side.getOppositeSide());
         desiredPositions.set(side, new YoFramePoint("finalDesiredPosition", side.getCamelCaseNameForMiddleOfExpression(), groundFrame, registry));
         desiredOrientations.set(side, new YoFrameOrientation("finalDesiredOrientation", side.getCamelCaseNameForMiddleOfExpression(), groundFrame, registry));
         
         desiredJointAngles.put(side, new LegJointPositions(side));
         desiredJointVelocities.put(side, new LegJointVelocities(legJointNames, side));
         
         
      }

      for(LegJointName jointName : legConfigurationData.getAllJoints())
      {
         legTorquesAtBeginningOfStep.put(jointName, new DoubleYoVariable(jointName.getCamelCaseNameForStartOfExpression() + "TorqueAtBeginningOfStep", registry));

         
//         filteredJointTorques.put(jointName, new AlphaFilteredYoVariable("alhpaFiltered"+jointName.getCamelCaseNameForMiddleOfExpression()+"Torque", registry, AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(15.0, controlDT)));
         
         filteredJointVelocities.put(jointName, new AlphaFilteredYoVariable("alhpaFiltered"+jointName.getCamelCaseNameForMiddleOfExpression()+"JointVelocity", registry, AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointVelocityBreakFrequency.getDoubleValue(), controlDT)));
      }
      
      parentRegistry.addChild(registry);

      setParameters();
   }

   
   private void resetFilters()
   {
      for(LegJointName jointName : legConfigurationData.getAllJoints())
      {
//         filteredJointTorques.get(jointName).reset();
         filteredJointVelocities.get(jointName).reset();
      }
   }
   private void setParameters()
   {
      swingDuration.set(0.65);
      compensateGravityForSwingLegTime.set(0.02);
      minimumTerminalSwingDuration.set(0.0);
      maximumTerminalSwingDuration.set(0.05);
      setEstimatedSwingTimeRemaining(swingDuration.getDoubleValue());
      ikAlpha.set(0.07);
      
      jointVelocityBreakFrequency.set(2.0);
   }


   public void doPreSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(swingDuration.getDoubleValue());
      
      gravityCompensationTrajectory.computeTrajectory(timeInState);
      double factor = gravityCompensationTrajectory.getPosition();
      
      
      torqueControlModule.computePreSwing(legTorquesToPackForSwingLeg);
      
      for(LegJointName legJointName : legJointNames)
      {
         double newTau = legTorquesToPackForSwingLeg.getTorque(legJointName);
         double oldTau = legTorquesAtBeginningOfStep.get(legJointName).getDoubleValue();
         
         double tau = (1.0 - factor) * oldTau + factor * newTau;
         
         legTorquesToPackForSwingLeg.setTorque(legJointName, tau);
         
      }
      
      couplingRegistry.getUpperBodyWrench().scale(factor);
      
      
      
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
      updateFinalDesiredPosition(legTorquesToPackForSwingLeg.getRobotSide());
      doSwing(legTorquesToPackForSwingLeg, timeInState, true);
      timeSpentInInitialSwing.set(timeInState);
   }

   public void doMidSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      updateFinalDesiredPosition(legTorquesToPackForSwingLeg.getRobotSide());
      doSwing(legTorquesToPackForSwingLeg, timeSpentInInitialSwing.getDoubleValue() + timeInState, true);
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
   
   private void computeDesiredAnglesAtEndOfSwing(RobotSide swingSide, double swingTimeRemaining, boolean useUpperBodyPositionAndVelocityEstimation)
   {
      
      FramePoint desiredPosition = desiredPositions.get(swingSide).getFramePointCopy();
      Orientation desiredOrientation = desiredOrientations.get(swingSide).getFrameOrientationCopy();
      
      
      
      Transform3D footToPelvis = computeDesiredTransform(referenceFrames.getPelvisFrame(), desiredPosition, desiredOrientation);

      
      if(useUpperBodyPositionAndVelocityEstimation)
      {
         Pair<FramePose, FrameVector> bodyPositionAndVelocityInTime = bodyPositionInTimeEstimator.getPelvisPoseAndPositionInTime(swingTimeRemaining, swingSide);
         FramePose pelvisPoseInTime = bodyPositionAndVelocityInTime.first();
         Transform3D pelvisTransformInTime = new Transform3D();
         pelvisPoseInTime.getTransform3D(pelvisTransformInTime);
         pelvisTransformInTime.invert();
         
         footToPelvis.mul(pelvisTransformInTime, footToPelvis);
         
         FrameVector upperBodyVelocityAtEndOfStep = bodyPositionAndVelocityInTime.second();
         ReferenceFrame footFrame = referenceFrames.getFootFrame(swingSide);
         Twist twistOfFootWithRespectToPelvis = new Twist(footFrame, ReferenceFrame.getWorldFrame(), footFrame);
         upperBodyVelocityAtEndOfStep.changeFrame(ReferenceFrame.getWorldFrame());
         twistOfFootWithRespectToPelvis.setAngularPart(upperBodyVelocityAtEndOfStep.getVector());
         
         
         desiredJointVelocityCalculators.get(swingSide).packDesiredJointVelocities(desiredJointVelocities.get(swingSide), twistOfFootWithRespectToPelvis, ikAlpha.getDoubleValue());
      }
      else
      {
         desiredJointVelocities.get(swingSide).setLegJointVelocitiesToDoubleArray(new double[legConfigurationData.getAllJoints().size()]);
         
      }
      
      
      try
      {
         inverseKinematicsCalculator.solve(desiredJointAngles.get(swingSide), footToPelvis, swingSide, desiredOrientation.getYawPitchRoll()[0]);
      } catch (InverseKinematicsException e)
      {
         numberOfInverseKinematicsErrors.increment();
      }
      
      

      

      
   }

   private void doSwing(LegTorques legTorques, double timeInSwing, boolean useUpperBodyPositionAndVelocityEstimation)
   {
      RobotSide robotSide = legTorques.getRobotSide();
      
      double swingTimeRemaining = swingDuration.getDoubleValue() - timeInSwing;
      if(swingTimeRemaining < 0.0)
         swingTimeRemaining = 0.0;
      
      legConfigurationData.setSwingTimeRemaining(swingTimeRemaining);
      legConfigurationData.setTimeInSwing(timeInSwing);
      legConfigurationData.setcurrentlyInSwing(true);
      setEstimatedSwingTimeRemaining(swingTimeRemaining);
      
      
      Orientation upperBodyOrientation = processedSensors.getPelvisOrientationInFrame(ReferenceFrame.getWorldFrame());
      legConfigurationData.setUpperBodyOrientationInWorld(upperBodyOrientation);
      
      
      FramePoint hipRollFramePoint = new FramePoint(referenceFrames.getLegJointFrames(robotSide).get(LegJointName.HIP_ROLL));
      hipRollFramePoint.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide.getOppositeSide()));
      
      legConfigurationData.setHipRollHeight(hipRollFramePoint.getZ());
      
      
      computeDesiredAnglesAtEndOfSwing(robotSide, swingTimeRemaining, useUpperBodyPositionAndVelocityEstimation);

      
      LegJointPositions finalDesiredLegJointPositions = desiredJointAngles.get(robotSide);
      LegJointVelocities finalDesiredLegJointVelocities = desiredJointVelocities.get(robotSide);
      
      
      // Do stupid stuff for hip yaw
      
      
      Orientation currentFootOrientation = new Orientation(referenceFrames.getAnkleZUpFrame(robotSide));
      ReferenceFrame groundPlaneFrame = desiredOrientations.get(robotSide).getReferenceFrame();
      currentFootOrientation.changeFrame(groundPlaneFrame);
      
      
      double initialYaw = currentFootOrientation.getYawPitchRoll()[0];
      
      Orientation desiredFootOrientation = desiredOrientations.get(robotSide).getFrameOrientationCopy();
      desiredFootOrientation.changeFrame(groundPlaneFrame);
      
      double finalYaw = desiredFootOrientation.getYawPitchRoll()[0];
      
      legConfigurationData.setCurrentJointAngle(LegJointName.HIP_YAW, initialYaw);
      legConfigurationData.setFinalDesiredJointAngle(LegJointName.HIP_YAW, finalYaw);
      
      
      for(LegJointName jointName : legConfigurationData.getAllJoints())
      {
         if(jointName == LegJointName.HIP_YAW)
         {
            continue;
         }
        
         
         legConfigurationData.setCurrentJointAngle(jointName, processedSensors.getLegJointPosition(robotSide, jointName));
         legConfigurationData.setFinalDesiredJointAngle(jointName, finalDesiredLegJointPositions.getJointPosition(jointName));
        
         filteredJointVelocities.get(jointName).setAlpha(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(jointVelocityBreakFrequency.getDoubleValue(), controlDT));
         filteredJointVelocities.get(jointName).update(processedSensors.getLegJointVelocity(robotSide, jointName));
         legConfigurationData.setCurrentJointVelocity(jointName, filteredJointVelocities.get(jointName).getDoubleValue());
      }
      
      for(LegJointName jointName : legConfigurationData.getJointsToOptimize())
      {
         if(useUpperBodyPositionAndVelocityEstimation)
         {
            legConfigurationData.setFinalDesiredJointVelocity(jointName, finalDesiredLegJointVelocities.getJointVelocity(jointName));
         }
         else
         {
            legConfigurationData.setFinalDesiredJointVelocity(jointName, 0.0);
         }
      }
      
      if(legTorqueData.isDataValid())
      {
      
         LegJointPositions legJointPositions = new LegJointPositions(robotSide);
         LegJointVelocities legJointVelocities = new LegJointVelocities(legJointNames, robotSide);
         LegJointAccelerations legJointAccelerations = new LegJointAccelerations(legJointNames, robotSide);
         
   
         
         for(LegJointName jointName : legTorqueData.getJointNames())
         {
            legJointPositions.setJointPosition(jointName, legTorqueData.getDesiredJointPosition(jointName));
            legJointVelocities.setJointVelocity(jointName, legTorqueData.getDesiredJointVelocity(jointName));
            legJointAccelerations.setJointAcceleration(jointName, legTorqueData.getDesiredJointAcceleration(jointName));
         }
   
         Orientation footOrientation = new Orientation(groundPlaneFrame, legJointPositions.getJointPosition(LegJointName.HIP_YAW), 0.0, 0.0);
         footOrientation.changeFrame(referenceFrames.getPelvisFrame());
         
         legJointPositions.setJointPosition(LegJointName.HIP_YAW, footOrientation.getYawPitchRoll()[0]);
         
         // Use craig300 for now, change to only ID
         // TODO: Get rid of these HACKS
         ((CraigPage300SwingLegTorqueControlOnlyModule) torqueControlModule).setParametersForOptimalSwing();
         torqueControlModule.compute(legTorques, legJointPositions, legJointVelocities, legJointAccelerations);
         
         

         
         
      }
      
//      for(LegJointName jointName : legConfigurationData.getAllJoints())
//      {
//         filteredJointTorques.get(jointName).update(legTorques.getTorque(jointName));
//         legTorques.setTorque(jointName, filteredJointTorques.get(jointName).getDoubleValue());
//      }

   }
   
   private void holdPosition(LegTorques legTorques)
   {
      RobotSide robotSide = legTorques.getRobotSide();
      computeDesiredAnglesAtEndOfSwing(robotSide, 0.0, false);
      
      LegJointPositions legJointPositions = desiredJointAngles.get(robotSide);
      LegJointVelocities legJointVelocities = desiredJointVelocities.get(robotSide);
      LegJointAccelerations legJointAccelerations = new LegJointAccelerations(legJointNames, robotSide);
      
      for(LegJointName jointName : legJointNames)
      {
//         legJointVelocities.setJointVelocity(jointName, 0.0);
         legJointAccelerations.setJointAcceleration(jointName, 0.0);
      }
      // TODO: Get rid of these hacks
      ((CraigPage300SwingLegTorqueControlOnlyModule) torqueControlModule).setParametersForM2V2();
      torqueControlModule.compute(legTorques, legJointPositions, legJointVelocities, legJointAccelerations);

   }

   public void doTerminalSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(0.0);

      doSwing(legTorquesToPackForSwingLeg, swingDuration.getDoubleValue(), true);
//      holdPosition(legTorquesToPackForSwingLeg);

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
      boolean footOnGround = footSwitches.get(swingSide).hasFootHitGround();

      boolean minimumTerminalSwingTimePassed = (timeInState > minimumTerminalSwingDuration.getDoubleValue());
      boolean capturePointInsideSupportFoot = isCapturePointInsideFoot(swingSide.getOppositeSide());

      if (capturePointInsideSupportFoot) return false; // Don't go in double support if ICP is still in support foot.
      
      return (footOnGround && minimumTerminalSwingTimePassed);

   }

   public boolean isDoneWithSwingInAir(double timeInState)
   {
      return (timeInState > 2.0*swingDuration.getDoubleValue());
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
      for(LegJointName jointName : legJointNames)
      {
         legTorquesAtBeginningOfStep.get(jointName).set(processedOutputs.getDesiredLegJointTorque(swingSide, jointName));
      }
      gravityCompensationTrajectory.setParams(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, compensateGravityForSwingLegTime.getDoubleValue());

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
      
      footSwitches.get(swingLeg).reset();
      resetFilters();

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
      
      FramePoint point = currentConfiguration.getDesiredSwingFootPosition();
      desiredPositions.get(swingLeg).set(point);
      desiredOrientations.get(swingLeg).setYawPitchRoll(0.0, 0.0, 0.0);
      
      resetFilters();

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
      updatePositionError(swingSide);
      legConfigurationData.setcurrentlyInSwing(false);
   }

   private void updatePositionError(RobotSide swingSide)
   {
      FramePoint currentPosition = new FramePoint(referenceFrames.getAnkleZUpFrame(swingSide));
      currentPosition.changeFrame(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
      FramePoint desiredPosition = desiredPositions.get(swingSide).getFramePointCopy();
      positionErrorAtEndOfStepNorm.set(desiredPosition.distance(currentPosition));
      currentPosition.sub(desiredPosition);
      positionErrorAtEndOfStepX.set(currentPosition.getX());
      positionErrorAtEndOfStepY.set(currentPosition.getY());
   }

   public void doTransitionOutOfSwingInAir(RobotSide swingLeg)
   {
      updatePositionError(swingLeg);
      legConfigurationData.setcurrentlyInSwing(false);
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


   public void doPreSwingInAir(LegTorques legTorques, double timeInState)
   {
      // TODO Auto-generated method stub
      
   }


   public boolean isDoneWithPreSwingInAir(RobotSide swingSide, double timeInState)
   {
      return true;
   }


   public void doTransitionIntoPreSwingInAir(RobotSide swingSide)
   {
      // TODO Auto-generated method stub
      
   }


   public void doTransitionOutOfPreSwingInAir(RobotSide swingLeg)
   {
      // TODO Auto-generated method stub
      
   }

}
