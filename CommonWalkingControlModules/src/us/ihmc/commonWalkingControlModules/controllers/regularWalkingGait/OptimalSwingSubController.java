package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.configurations.BalanceOnOneLegConfiguration;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlOnlyModule;
import us.ihmc.commonWalkingControlModules.controlModules.LegJointPositionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.swingLegTorqueControl.CraigPage300SwingLegTorqueControlOnlyModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.kinematics.SwingLegAnglesAtEndOfStepEstimator;
import us.ihmc.commonWalkingControlModules.optimalSwing.LegTorqueData;
import us.ihmc.commonWalkingControlModules.optimalSwing.SwingParameters;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.graphics.ArtifactList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.splines.QuinticSplineInterpolator;
import com.yobotics.simulationconstructionset.util.trajectory.YoMinimumJerkTrajectory;

public class OptimalSwingSubController implements SwingSubController
{
   private static final LegJointName[] legJointNames = new LegJointName[] { LegJointName.HIP_YAW, LegJointName.HIP_PITCH, LegJointName.HIP_ROLL,
         LegJointName.KNEE, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL };

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final CommonWalkingReferenceFrames referenceFrames;
   private final CouplingRegistry couplingRegistry;
   private final ProcessedSensorsInterface processedSensors;
   private final ProcessedOutputsInterface processedOutputs;
   private final DesiredFootstepCalculator desiredFootstepCalculator;

   private final SwingParameters swingParameters;
   private final LegTorqueData legTorqueData;

   private final EnumMap<LegJointName, DoubleYoVariable> legTorquesAtBeginningOfStep = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final YoMinimumJerkTrajectory gravityCompensationTrajectory = new YoMinimumJerkTrajectory("gravityCompensationTrajectory", registry);

//   private final DoubleYoVariable swingDuration = new DoubleYoVariable("swingDuration", "The duration of the swing movement. [s]", registry);

   private final SideDependentList<YoFramePoint> desiredPositions = new SideDependentList<YoFramePoint>();
   private final YoFramePoint desiredPositionInWorld = new YoFramePoint("desiredPositionInWorld", ReferenceFrame.getWorldFrame(), registry);
   private final SideDependentList<YoFrameOrientation> desiredOrientations = new SideDependentList<YoFrameOrientation>();

   private final SideDependentList<LegJointPositions> desiredJointAngles = new SideDependentList<LegJointPositions>();
   private final SideDependentList<LegJointVelocities> desiredJointVelocities = new SideDependentList<LegJointVelocities>();

   private final DoubleYoVariable timeSpentInPreSwing = new DoubleYoVariable("timeSpentInPreSwing", "This is the time spent in Pre swing.", registry);
   private final DoubleYoVariable timeSpentInInitialSwing = new DoubleYoVariable("timeSpentInInitialSwing", "This is the time spent in initial swing.",
         registry);
   private final DoubleYoVariable timeSpentInMidSwing = new DoubleYoVariable("timeSpentInMidSwing", "This is the time spend in mid swing.", registry);
   private final DoubleYoVariable timeSpentInTerminalSwing = new DoubleYoVariable("timeSpentInTerminalSwing", "This is the time spent in terminal swing.",
         registry);

   private final DoubleYoVariable minimumTerminalSwingDuration = new DoubleYoVariable("minimumTerminalSwingDuration",
         "The minimum duration of terminal swing state. [s]", registry);
   private final DoubleYoVariable maximumTerminalSwingDuration = new DoubleYoVariable("maximumTerminalSwingDuration",
         "The maximum duration of terminal swing state. [s]", registry);

   private final DoubleYoVariable compensateGravityForSwingLegTime = new DoubleYoVariable("compensateGravityForSwingLegTime", registry);

   private final DoubleYoVariable estimatedSwingTimeRemaining = new DoubleYoVariable("estimatedSwingTimeRemaining", registry);

   private final BooleanYoVariable canGoToDoubleSupportFromLastTickState = new BooleanYoVariable("canGoToDoubleSupportFromLastTickState", registry);

   private final DoubleYoVariable positionErrorAtEndOfStepNorm = new DoubleYoVariable("positionErrorAtEndOfStepNorm", registry);
   private final DoubleYoVariable positionErrorAtEndOfStepX = new DoubleYoVariable("positionErrorAtEndOfStepX", registry);
   private final DoubleYoVariable positionErrorAtEndOfStepY = new DoubleYoVariable("positionErrorAtEndOfStepY", registry);
   private final SwingLegTorqueControlOnlyModule torqueControlModule;

   private final DoubleYoVariable initialHipYawAngle = new DoubleYoVariable("intialHipYawAngle", registry);
   private final YoFrameOrientation initialAnkleOrientation;
   private final QuinticSplineInterpolator positionInterpolator = new QuinticSplineInterpolator("hipYawInterpolator", 2, 1, registry);

   private final PDController hipYawAngleController = new PDController("HipYawAngleController", registry);
   private final PDController ankleRollController = new PDController("AnkleRollController", registry);
   private final PDController anklePitchController = new PDController("AnklePitchController", registry);

   private final DoubleYoVariable ikAlpha = new DoubleYoVariable("ikAlpha", registry);

   //   private final EnumMap<LegJointName, AlphaFilteredYoVariable> filteredJointTorques = new EnumMap<LegJointName, AlphaFilteredYoVariable>(LegJointName.class);

   private final DoubleYoVariable desiredAccelerationBreakFrequency = new DoubleYoVariable("desiredAccelerationBreakFrequency", registry);
   private final EnumMap<LegJointName, AlphaFilteredYoVariable> filteredDesiredJointAccelerations = new EnumMap<LegJointName, AlphaFilteredYoVariable>(
         LegJointName.class);
   
   private final YoFrameVector positionInSupportLegAnkleZUp = new YoFrameVector("positionInSupportLegAnkleZUp", ReferenceFrame.getWorldFrame(), registry);

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final SwingLegAnglesAtEndOfStepEstimator swingLegAnglesAtEndOfStepEstimator;

   private final ReferenceFrame desiredHeadingFrame;

//   private final double controlDT;

   private final BooleanYoVariable useUpperBodyPositionEstimation = new BooleanYoVariable("useUpperBodyPositionEstimation", registry);

   public OptimalSwingSubController(ProcessedSensorsInterface processedSensors, ProcessedOutputsInterface processedOutputs,
         CommonWalkingReferenceFrames referenceFrames, FullRobotModel fullRobotModel, DesiredFootstepCalculator desiredFootstepCalculator,
         SideDependentList<FootSwitchInterface> footSwitches, CouplingRegistry couplingRegistry, SwingParameters swingParameters, LegTorqueData legTorqueData,
         SwingLegTorqueControlOnlyModule swingLegTorqueControlModule, DesiredHeadingControlModule desiredHeadingControlModule,
         SideDependentList<LegJointPositionControlModule> legJointPositionControlModules, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
         SwingLegAnglesAtEndOfStepEstimator swingLegAnglesAtEndOfStepEstimator, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      this.couplingRegistry = couplingRegistry;
      this.processedSensors = processedSensors;
      this.processedOutputs = processedOutputs;
      this.torqueControlModule = swingLegTorqueControlModule;
      this.swingLegAnglesAtEndOfStepEstimator = swingLegAnglesAtEndOfStepEstimator;
      this.desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();
//      this.controlDT = controlDT;

      this.swingParameters = swingParameters;
      this.legTorqueData = legTorqueData;

      this.footSwitches = footSwitches;

      for (RobotSide side : RobotSide.values)
      {
         ReferenceFrame groundFrame = referenceFrames.getAnkleZUpFrame(side.getOppositeSide());
         desiredPositions.set(side, new YoFramePoint("finalDesiredPosition", side.getCamelCaseNameForMiddleOfExpression(), groundFrame, registry));
         desiredOrientations.set(side, new YoFrameOrientation("finalDesiredOrientation", side.getCamelCaseNameForMiddleOfExpression(), groundFrame, registry));

         desiredJointAngles.put(side, new LegJointPositions(side));
         desiredJointVelocities.put(side, new LegJointVelocities(legJointNames, side));

      }

      for (LegJointName jointName : legJointNames)
      {
         legTorquesAtBeginningOfStep.put(jointName,
               new DoubleYoVariable(jointName.getCamelCaseNameForStartOfExpression() + "TorqueAtBeginningOfStep", registry));

         filteredDesiredJointAccelerations.put(jointName,
               new AlphaFilteredYoVariable("alhpaFiltered" + jointName.getCamelCaseNameForMiddleOfExpression() + "DesiredJointAcceleration", registry,
                     AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(desiredAccelerationBreakFrequency.getDoubleValue(), controlDT)));
      }

      initialAnkleOrientation = new YoFrameOrientation("initialAnkleOrientation", "", desiredHeadingFrame, registry);

      if(dynamicGraphicObjectsListRegistry != null)
      {
         ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());
         ArrayList<DynamicGraphicObject> dynamicGraphicObjects = new ArrayList<DynamicGraphicObject>();

         

         
         DynamicGraphicPosition finalDesiredViz = desiredPositionInWorld.createDynamicGraphicPosition("Final Desired Swing", 0.04,
               YoAppearance.Purple(), GraphicType.BALL);
         
         

         artifactList.add(finalDesiredViz.createArtifact());
         dynamicGraphicObjects.add(finalDesiredViz);

         

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("OptimalSwingSubController", dynamicGraphicObjects);

         dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
      }
      
      parentRegistry.addChild(registry);

      setParameters();
   }

   public void setUseUpperBodyPositionEstimation(boolean val)
   {
      useUpperBodyPositionEstimation.set(val);
   }
   
   private void resetFilters()
   {
      for (LegJointName jointName : legJointNames)
      {
         //         filteredJointTorques.get(jointName).reset();
         filteredDesiredJointAccelerations.get(jointName).reset();
      }
   }
   private double getSwingDuration()
   {
      return couplingRegistry.getSingleSupportDuration();
   }

   private void setParameters()
   {
      compensateGravityForSwingLegTime.set(0.02);
      minimumTerminalSwingDuration.set(0.0);
      maximumTerminalSwingDuration.set(0.05);
      setEstimatedSwingTimeRemaining(getSwingDuration());
      ikAlpha.set(0.07);

      desiredAccelerationBreakFrequency.set(25.0);

      hipYawAngleController.setProportionalGain(120.0);
      hipYawAngleController.setDerivativeGain(2.0);

      anklePitchController.setProportionalGain(200.0);
      anklePitchController.setDerivativeGain(2.0);

      ankleRollController.setProportionalGain(40.0);
      ankleRollController.setDerivativeGain(2.0);
      
      useUpperBodyPositionEstimation.set(true);
   }

   public void doPreSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(getSwingDuration());

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

   private void updateFinalDesiredPosition(RobotSide swingLeg)
   {
      FramePose desiredFootstepPose = couplingRegistry.getDesiredFootstep().getPoseCopy();

      FramePoint desiredSwingFootPosition = new FramePoint();
      desiredFootstepPose.getPosition(desiredSwingFootPosition);
      desiredSwingFootPosition.changeFrame(desiredPositions.get(swingLeg).getReferenceFrame());
      FrameOrientation desiredSwingFootOrientation = desiredFootstepPose.getOrientationCopy().changeFrameCopy(desiredOrientations.get(swingLeg).getReferenceFrame());

      desiredPositions.get(swingLeg).set(desiredSwingFootPosition);
      desiredOrientations.get(swingLeg).set(desiredSwingFootOrientation);
      
      desiredSwingFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
      desiredPositionInWorld.set(desiredSwingFootPosition);

   }

   public void doInitialSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      updateFinalDesiredPosition(legTorquesToPackForSwingLeg.getRobotSide());
      doSwing(legTorquesToPackForSwingLeg, timeInState, false);
      timeSpentInInitialSwing.set(timeInState);
   }

   public void doMidSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      updateFinalDesiredPosition(legTorquesToPackForSwingLeg.getRobotSide());
      doSwing(legTorquesToPackForSwingLeg, timeSpentInInitialSwing.getDoubleValue() + timeInState, useUpperBodyPositionEstimation.getBooleanValue());
      timeSpentInMidSwing.set(timeInState);
   }

   private void computeDesiredAnglesAtEndOfSwing(RobotSide swingSide, double swingTimeRemaining, boolean useUpperBodyPositionAndVelocityEstimation)
   {

      FramePoint desiredPosition = desiredPositions.get(swingSide).getFramePointCopy();
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      desiredPosition.changeFrame(pelvisFrame);
      FrameOrientation desiredOrientation = desiredOrientations.get(swingSide).getFrameOrientationCopy();
      desiredOrientation.changeFrame(pelvisFrame);

      double desiredYaw = desiredOrientation.getYawPitchRoll()[0];

      swingLegAnglesAtEndOfStepEstimator.getEstimatedJointAnglesAtEndOfStep(desiredJointAngles.get(swingSide), desiredJointVelocities.get(swingSide),
            swingSide, desiredPosition, desiredOrientation, desiredYaw, swingTimeRemaining, useUpperBodyPositionAndVelocityEstimation);

   }

   public void doTransitionIntoSwing(RobotSide swingLeg)
   {

      swingParameters.setRobotSide(swingLeg);
      FrameOrientation currentFootOrientation = new FrameOrientation(referenceFrames.getAnkleZUpFrame(swingLeg));
      currentFootOrientation.changeFrame(desiredHeadingFrame);

      initialHipYawAngle.set(currentFootOrientation.getYawPitchRoll()[0]);
      initialAnkleOrientation.set(currentFootOrientation);
   }
   
   private void updateGroundClearance(RobotSide robotSide)
   {
      FramePoint swingFootPoint = new FramePoint(referenceFrames.getAnkleZUpFrame(robotSide));
      swingFootPoint.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide.getOppositeSide()));
      
//      FrameVector swingFootVector = new FrameVector(swingFootPoint);
//      swingFootVector.changeFrame(ReferenceFrame.getWorldFrame());
      
      positionInSupportLegAnkleZUp.set(swingFootPoint.getVectorCopy());
   }

   private void doSwing(LegTorques legTorques, double timeInSwing, boolean useUpperBodyPositionAndVelocityEstimation)
   {
      RobotSide robotSide = legTorques.getRobotSide();
      updateGroundClearance(robotSide);
      double swingTimeRemaining = getSwingDuration() - timeInSwing;
      if (swingTimeRemaining < 0.0)
         swingTimeRemaining = 0.0;

      swingParameters.setTotalSwingTime(getSwingDuration());
      swingParameters.setSwingTimeRemaining(swingTimeRemaining);
      swingParameters.setCurrentlyInSwing(true);
      setEstimatedSwingTimeRemaining(swingTimeRemaining);

      computeDesiredAnglesAtEndOfSwing(robotSide, swingTimeRemaining, useUpperBodyPositionAndVelocityEstimation);

      LegJointPositions finalDesiredLegJointPositions = desiredJointAngles.get(robotSide);
      LegJointVelocities finalDesiredLegJointVelocities = desiredJointVelocities.get(robotSide);
      //      FramePoint desiredStepLocation = desiredPositions.get(robotSide).getFramePointCopy();
      //      desiredStepLocation.changeFrame(referenceFrames.getLegJointFrame(robotSide, LegJointName.HIP_YAW));
      //      swingParameters.setDesiredStepLocationInHipYawFrame(desiredStepLocation.getPoint());

      //      FramePoint currentFootLocation = new FramePoint(referenceFrames.getAnkleZUpFrame(robotSide));
      //      currentFootLocation.changeFrame(referenceFrames.getLegJointFrame(robotSide, LegJointName.HIP_YAW));
      //      swingParameters.setcurrentFootLocation(currentFootLocation.getPoint());

      //      FramePoint hip = new FramePoint(referenceFrames.getLegJointFrame(robotSide, LegJointName.HIP_YAW));
      //      hip.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide));
      //      
      //      swingParameters.setHipHeight(hip.getZ());

      // Control hip yaw angle in desiredHeadingFrame

      for (LegJointName jointName : legJointNames)
      {
         swingParameters.setDesiredJointPosition(robotSide, jointName, finalDesiredLegJointPositions.getJointPosition(jointName));
         swingParameters.setDesiredJointVelocity(robotSide, jointName, finalDesiredLegJointVelocities.getJointVelocity(jointName));
      }

      if (legTorqueData.isDataValid())
      {

         LegJointPositions legJointPositions = new LegJointPositions(robotSide);
         LegJointVelocities legJointVelocities = new LegJointVelocities(legJointNames, robotSide);
         LegJointAccelerations legJointAccelerations = new LegJointAccelerations(legJointNames, robotSide);

         for (LegJointName jointName : legTorqueData.getJointNames())
         {
            legJointPositions.setJointPosition(jointName, legTorqueData.getDesiredJointPosition(jointName));
            legJointVelocities.setJointVelocity(jointName, legTorqueData.getDesiredJointVelocity(jointName));

//            filteredDesiredJointAccelerations.get(jointName).setAlpha(
//                  AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(desiredAccelerationBreakFrequency.getDoubleValue(), controlDT));
//            filteredDesiredJointAccelerations.get(jointName).update(legTorqueData.getDesiredJointAcceleration(jointName));

            legJointAccelerations.setJointAcceleration(jointName, legTorqueData.getDesiredJointAcceleration(jointName)); //filteredDesiredJointAccelerations.get(jointName).getDoubleValue());
         }
         //   
         //         Orientation footOrientation = new Orientation(groundPlaneFrame, legJointPositions.getJointPosition(LegJointName.HIP_YAW), 0.0, 0.0);
         //         footOrientation.changeFrame(referenceFrames.getPelvisFrame());
         //         
         //         legJointPositions.setJointPosition(LegJointName.HIP_YAW, footOrientation.getYawPitchRoll()[0]);
         //         
         // Use craig300 for now, change to only ID
         // TODO: Get rid of these HACKS
         ((CraigPage300SwingLegTorqueControlOnlyModule) torqueControlModule).setParametersForOptimalSwing();
         torqueControlModule.compute(legTorques, legJointPositions, legJointVelocities, legJointAccelerations);

      }

      /*
       * Hip yaw control is really bad, so control it with a PD controller
       */

      FrameOrientation desiredFootOrientation = desiredOrientations.get(robotSide).getFrameOrientationCopy();
      desiredFootOrientation.changeFrame(desiredHeadingFrame);
      double[] finalFootYawPitchRoll = desiredFootOrientation.getYawPitchRoll();
      double finalFootYaw = finalFootYawPitchRoll[0];

      double t[] = { 0, getSwingDuration() };
      //      double yIn[] = { initialHipYawAngle.getDoubleValue(), finalFootYaw };
      double yIn[] = { 0.0, 1.0 };

      positionInterpolator.initialize(t);
      positionInterpolator.determineCoefficients(0, yIn, 0.0, 0.0, 0.0, 0.0);

      double[][] positionResult = new double[1][2];
      positionInterpolator.compute(timeInSwing, 1, positionResult);

      double desiredHipYawOrientationAngle = (finalFootYaw - initialHipYawAngle.getDoubleValue()) * positionResult[0][0] + initialHipYawAngle.getDoubleValue();
      FrameOrientation hipYawOrientation = new FrameOrientation(desiredHeadingFrame, desiredHipYawOrientationAngle, 0.0, 0.0);
      hipYawOrientation.changeFrame(referenceFrames.getPelvisFrame());
      double desiredHipYawAngle = hipYawOrientation.getYawPitchRoll()[0];

      double desiredHipYawVelocity = 0.0;

      double hipYawTorque = hipYawAngleController.compute(processedSensors.getLegJointPosition(robotSide, LegJointName.HIP_YAW), desiredHipYawAngle,
            processedSensors.getLegJointVelocity(robotSide, LegJointName.HIP_YAW), desiredHipYawVelocity);

      legTorques.setTorque(LegJointName.HIP_YAW, hipYawTorque);

      /*
       * HACK: Control ankle orientation
       */

      ReferenceFrame beforeAnklePitchFrame = referenceFrames.getLegJointFrame(robotSide, LegJointName.KNEE);

      FrameOrientation currentDesiredFootOrientation = new FrameOrientation(desiredHeadingFrame);
      double desiredHipPitchAngle = (finalFootYawPitchRoll[1] - initialAnkleOrientation.getPitch().getDoubleValue()) * positionResult[0][0]
            + initialAnkleOrientation.getPitch().getDoubleValue();
      double desiredHipRollAngle = (finalFootYawPitchRoll[2] - initialAnkleOrientation.getRoll().getDoubleValue()) * positionResult[0][0] 
            + initialAnkleOrientation.getRoll().getDoubleValue();
      currentDesiredFootOrientation.setYawPitchRoll(desiredHipYawAngle, desiredHipPitchAngle, desiredHipRollAngle);
      currentDesiredFootOrientation.changeFrame(beforeAnklePitchFrame);
      double[] yawPitchRoll = currentDesiredFootOrientation.getYawPitchRoll();

      double anklePitchTorque = anklePitchController.computeForAngles(processedSensors.getLegJointPosition(robotSide, LegJointName.ANKLE_PITCH),
            yawPitchRoll[1], processedSensors.getLegJointVelocity(robotSide, LegJointName.ANKLE_PITCH), 0.0);
      double ankleRollTorque = ankleRollController.computeForAngles(processedSensors.getLegJointPosition(robotSide, LegJointName.ANKLE_ROLL), yawPitchRoll[2],
            processedSensors.getLegJointVelocity(robotSide, LegJointName.ANKLE_ROLL), 0.0);

      legTorques.setTorque(LegJointName.ANKLE_PITCH, anklePitchTorque);
      legTorques.setTorque(LegJointName.ANKLE_ROLL, ankleRollTorque);

   }

   //   private void holdPosition(LegTorques legTorques)
   //   {
   //      RobotSide robotSide = legTorques.getRobotSide();
   //      computeDesiredAnglesAtEndOfSwing(robotSide, 0.0, false);
   //      
   //      LegJointPositions legJointPositions = desiredJointAngles.get(robotSide);
   //      LegJointVelocities legJointVelocities = desiredJointVelocities.get(robotSide);
   //      LegJointAccelerations legJointAccelerations = new LegJointAccelerations(legJointNames, robotSide);
   //      
   //      for(LegJointName jointName : legJointNames)
   //      {
   ////         legJointVelocities.setJointVelocity(jointName, 0.0);
   //         legJointAccelerations.setJointAcceleration(jointName, 0.0);
   //      }
   //      // TODO: Get rid of these hacks
   //      ((CraigPage300SwingLegTorqueControlOnlyModule) torqueControlModule).setParametersForM2V2();
   //      torqueControlModule.compute(legTorques, legJointPositions, legJointVelocities, legJointAccelerations);
   //
   //   }

   public void doTerminalSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(0.0);
      updateFinalDesiredPosition(legTorquesToPackForSwingLeg.getRobotSide());
      doSwing(legTorquesToPackForSwingLeg, getSwingDuration(), useUpperBodyPositionEstimation.getBooleanValue());
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

      return inStateLongEnough || isCoMPastSweetSpot;
   }

   public boolean isDoneWithMidSwing(RobotSide swingSide, double timeInState)
   {
      return timeSpentInInitialSwing.getDoubleValue() + timeInState > getSwingDuration() || isDoneWithTerminalSwing(swingSide, timeInState);
   }

   public boolean isDoneWithTerminalSwing(RobotSide swingSide, double timeInState)
   {
      boolean footOnGround = footSwitches.get(swingSide).hasFootHitGround();
      boolean footOnGroundByHeight = positionInSupportLegAnkleZUp.getZ() < 0.005;
      

      boolean minimumTerminalSwingTimePassed = (timeInState > minimumTerminalSwingDuration.getDoubleValue());
      boolean capturePointInsideSupportFoot = isCapturePointInsideFoot(swingSide.getOppositeSide());

      if (capturePointInsideSupportFoot)
         return false; // Don't go in double support if ICP is still in support foot.

      return ((footOnGround || footOnGroundByHeight) && minimumTerminalSwingTimePassed);

   }

   public boolean isDoneWithSwingInAir(double timeInState)
   {
      return (timeInState > 2.0 * getSwingDuration());
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

   public void doTransitionIntoInitialSwing(RobotSide swingLeg)
   {
      Footstep desiredFootstep = couplingRegistry.getDesiredFootstep();

      FramePose desiredFootstepPose = desiredFootstep.getPoseCopy();

      FramePoint endPoint = new FramePoint();
      desiredFootstepPose.getPosition(endPoint);
      endPoint.changeFrame(desiredPositions.get(swingLeg).getReferenceFrame());
      FrameOrientation endOrientation = new FrameOrientation(desiredFootstepPose.getOrientationCopy());
      endOrientation.changeFrame(desiredOrientations.get(swingLeg).getReferenceFrame());

      // Setup the orientation trajectory
      desiredPositions.get(swingLeg).set(endPoint);
      desiredOrientations.get(swingLeg).set(endOrientation);

      footSwitches.get(swingLeg).reset();
      resetFilters();

      doTransitionIntoSwing(swingLeg);

   }

   public void doTransitionIntoMidSwing(RobotSide swingSide)
   {
   }

   public void doTransitionIntoTerminalSwing(RobotSide swingSide)
   {
   }

   public void doTransitionIntoSwingInAir(RobotSide swingLeg, BalanceOnOneLegConfiguration currentConfiguration)
   {

      FramePoint point = currentConfiguration.getDesiredSwingFootPosition();
      desiredPositions.get(swingLeg).set(point);
      desiredOrientations.get(swingLeg).setYawPitchRoll(0.0, 0.0, 0.0);

      resetFilters();

      doTransitionIntoSwing(swingLeg);
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
      swingParameters.setCurrentlyInSwing(false);
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
      swingParameters.setCurrentlyInSwing(false);
      
      desiredFootstepCalculator.initializeDesiredFootstep(swingLeg.getOppositeSide());
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
      return getSwingDuration() < timeInState;
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
