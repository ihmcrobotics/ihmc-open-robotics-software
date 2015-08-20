package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.configurations.BalanceOnOneLegConfiguration;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlOnlyModule;
import us.ihmc.commonWalkingControlModules.controlModules.LegJointPositionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.swingLegTorqueControl.CraigPage300SwingLegTorqueControlOnlyModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.kinematics.SwingLegAnglesAtEndOfStepEstimator;
import us.ihmc.commonWalkingControlModules.optimalSwing.LegTorqueData;
import us.ihmc.commonWalkingControlModules.optimalSwing.SwingParameters;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.controllers.PDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphic;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.interpolators.QuinticSplineInterpolator;
import us.ihmc.yoUtilities.math.trajectories.YoMinimumJerkTrajectory;


public class OptimalSwingSubController implements SwingSubController
{
   private static final LegJointName[] legJointNames = new LegJointName[] { LegJointName.HIP_YAW, LegJointName.HIP_PITCH, LegJointName.HIP_ROLL,
         LegJointName.KNEE, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL };

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final CouplingRegistry couplingRegistry;
   private final ProcessedSensorsInterface processedSensors;
   private final ProcessedOutputsInterface processedOutputs;
   private final DesiredFootstepCalculator desiredFootstepCalculator;

   private final SwingParameters swingParameters;
   private final LegTorqueData legTorqueData;

   private final EnumMap<LegJointName, DoubleYoVariable> legTorquesAtBeginningOfStep = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final YoMinimumJerkTrajectory gravityCompensationTrajectory = new YoMinimumJerkTrajectory("gravityCompensationTrajectory", registry);

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

   private final DoubleYoVariable desiredAccelerationBreakFrequency = new DoubleYoVariable("desiredAccelerationBreakFrequency", registry);
   private final EnumMap<LegJointName, AlphaFilteredYoVariable> filteredDesiredJointAccelerations = new EnumMap<LegJointName, AlphaFilteredYoVariable>(
         LegJointName.class);

   private final YoFrameVector positionInSupportLegAnkleZUp = new YoFrameVector("positionInSupportLegAnkleZUp", ReferenceFrame.getWorldFrame(), registry);

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final SwingLegAnglesAtEndOfStepEstimator swingLegAnglesAtEndOfStepEstimator;

   private final ReferenceFrame desiredHeadingFrame;

   private final BooleanYoVariable useUpperBodyPositionEstimation = new BooleanYoVariable("useUpperBodyPositionEstimation", registry);

   public OptimalSwingSubController(ProcessedSensorsInterface processedSensors, ProcessedOutputsInterface processedOutputs,
         CommonHumanoidReferenceFrames referenceFrames, FullRobotModel fullRobotModel, DesiredFootstepCalculator desiredFootstepCalculator,
         SideDependentList<FootSwitchInterface> footSwitches, CouplingRegistry couplingRegistry, SwingParameters swingParameters, LegTorqueData legTorqueData,
         SwingLegTorqueControlOnlyModule swingLegTorqueControlModule, DesiredHeadingControlModule desiredHeadingControlModule,
         SideDependentList<LegJointPositionControlModule> legJointPositionControlModules, YoGraphicsListRegistry yoGraphicsListRegistry,
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

      if (yoGraphicsListRegistry != null)
      {
         ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());
         ArrayList<YoGraphic> yoGraphics = new ArrayList<YoGraphic>();

         YoGraphicPosition finalDesiredViz = new YoGraphicPosition("Final Desired Swing", desiredPositionInWorld, 0.04, YoAppearance.Purple(),
               GraphicType.BALL);

         artifactList.add(finalDesiredViz.createArtifact());
         yoGraphics.add(finalDesiredViz);

         yoGraphicsListRegistry.registerYoGraphics("OptimalSwingSubController", yoGraphics);

         yoGraphicsListRegistry.registerArtifactList(artifactList);
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
      FramePose desiredFootstepPose = new FramePose();
      couplingRegistry.getDesiredFootstep().getPose(desiredFootstepPose);

      FramePoint desiredSwingFootPosition = new FramePoint();
      desiredFootstepPose.getPositionIncludingFrame(desiredSwingFootPosition);
      desiredSwingFootPosition.changeFrame(desiredPositions.get(swingLeg).getReferenceFrame());
      FrameOrientation desiredSwingFootOrientation = new FrameOrientation();
      desiredFootstepPose.getOrientationIncludingFrame(desiredSwingFootOrientation);
      desiredSwingFootOrientation.changeFrame(desiredOrientations.get(swingLeg).getReferenceFrame());

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

            legJointAccelerations.setJointAcceleration(jointName, legTorqueData.getDesiredJointAcceleration(jointName)); //filteredDesiredJointAccelerations.get(jointName).getDoubleValue());
         }
     
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

   //      // TODO: Get rid of these hacks
   //      ((CraigPage300SwingLegTorqueControlOnlyModule) torqueControlModule).setParametersForM2V2();
   //      torqueControlModule.compute(legTorques, legJointPositions, legJointVelocities, legJointAccelerations);
   //   }

   public void doTerminalSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(0.0);
      updateFinalDesiredPosition(legTorquesToPackForSwingLeg.getRobotSide());
      doSwing(legTorquesToPackForSwingLeg, getSwingDuration(), useUpperBodyPositionEstimation.getBooleanValue());

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

      FramePose desiredFootstepPose = new FramePose();
      desiredFootstep.getPose(desiredFootstepPose);

      FramePoint endPoint = new FramePoint();
      desiredFootstepPose.getPositionIncludingFrame(endPoint);
      endPoint.changeFrame(desiredPositions.get(swingLeg).getReferenceFrame());
      FrameOrientation endOrientation = new FrameOrientation();
      desiredFootstepPose.getOrientationIncludingFrame(endOrientation);
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
      FrameConvexPolygon2d footPolygon = couplingRegistry.getOldBipedSupportPolygons().getFootPolygonInAnkleZUp(swingSide);
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