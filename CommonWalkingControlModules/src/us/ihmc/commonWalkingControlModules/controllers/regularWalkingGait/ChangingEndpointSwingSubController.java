package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.configurations.BalanceOnOneLegConfiguration;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PreSwingControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.AnkleVelocityCalculator;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.graphics.BagOfBalls;
import us.ihmc.yoUtilities.graphics.YoGraphic;
import us.ihmc.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.yoUtilities.math.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.YoMinimumJerkTrajectory;


public class ChangingEndpointSwingSubController implements SwingSubController
{
   private final ProcessedSensorsInterface processedSensors;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final CouplingRegistry couplingRegistry;

   private final DesiredFootstepCalculator desiredFootstepCalculator;

   private final CartesianTrajectoryGenerator walkingTrajectoryGenerator;
   private final SideDependentList<CartesianTrajectoryGenerator> swingInAirTrajectoryGenerators;

   private final SwingLegTorqueControlModule swingLegTorqueControlModule;

   private final SideDependentList<AnkleVelocityCalculator> ankleVelocityCalculators;
   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final PreSwingControlModule preSwingControlModule;

   private final YoVariableRegistry registry = new YoVariableRegistry("SwingSubConroller");

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final DoubleYoVariable passiveHipCollapseTime = new DoubleYoVariable("passiveHipCollapseTime", registry);

   private final DoubleYoVariable swingOrientationTime = new DoubleYoVariable("swingOrientationTime",
         "The duration of the foot orientation part of the swing.", registry);

   private final DoubleYoVariable initialSwingVelocity = new DoubleYoVariable("initialSwingVelocity", registry);
   private final DoubleYoVariable initialSwingAcceleration = new DoubleYoVariable("initialSwingAcceleration", registry);
   private final DoubleYoVariable finalSwingVelocity = new DoubleYoVariable("finalSwingVelocity", registry);
   private final DoubleYoVariable finalSwingAcceleration = new DoubleYoVariable("finalSwingAcceleration", registry);

   private final DoubleYoVariable minimumTerminalSwingDuration = new DoubleYoVariable("minimumTerminalSwingDuration",
         "The minimum duration of terminal swing state. [s]", registry);
   private final DoubleYoVariable maximumTerminalSwingDuration = new DoubleYoVariable("maximumTerminalSwingDuration",
         "The maximum duration of terminal swing state. [s]", registry);

   private final DoubleYoVariable terminalSwingGainRampTime = new DoubleYoVariable("terminalSwingGainRampTime", "The time to ramp the gains to zero [s]",
         registry);

   private final DoubleYoVariable estimatedSwingTimeRemaining = new DoubleYoVariable("estimatedSwingTimeRemaining", "The estimated Swing Time Remaining [s]",
         registry);
   private final DoubleYoVariable antiGravityPercentage = new DoubleYoVariable("antiGravityPercentage", "The percent of antigravity effort (0,1)", registry);

   private final DoubleYoVariable swingToePitchUpOnLanding = new DoubleYoVariable("swingToePitchUpOnLanding",
         "How much to pitch up the swing toe at the end of the swing.", registry);

   private final DoubleYoVariable comXThresholdToFinishInitialSwing = new DoubleYoVariable("comXThresholdToFinishInitialSwing",
         "How far the CoM should be in front of the support foot before transitioning out of initial swing.", registry);

   private final DoubleYoVariable timeSpentInPreSwing = new DoubleYoVariable("timeSpentInPreSwing", "This is the time spent in Pre swing.", registry);
   private final DoubleYoVariable timeSpentInInitialSwing = new DoubleYoVariable("timeSpentInInitialSwing", "This is the time spent in initial swing.",
         registry);
   private final DoubleYoVariable timeSpentInMidSwing = new DoubleYoVariable("timeSpentInMidSwing", "This is the time spend in mid swing.", registry);
   private final DoubleYoVariable timeSpentInTerminalSwing = new DoubleYoVariable("timeSpentInTerminalSwing", "This is the time spent in terminal swing.",
         registry);

   private final DoubleYoVariable swingFootPositionError = new DoubleYoVariable("swingFootPositionError", registry);

   private final YoMinimumJerkTrajectory minimumJerkTrajectoryForFootOrientation = new YoMinimumJerkTrajectory("swingFootOrientation", registry);

   private final YoFrameOrientation desiredFootOrientationInWorldFrame = new YoFrameOrientation("desiredFootOrientationInWorld", "", worldFrame, registry);
   private final YoFrameOrientation finalDesiredFootOrientationInWorldFrame = new YoFrameOrientation("finalDesiredFootOrientationInWorld", "", worldFrame,
         registry);
   private final SideDependentList<YoFrameOrientation> startSwingOrientations = new SideDependentList<YoFrameOrientation>();
   private final SideDependentList<YoFrameOrientation> endSwingOrientations = new SideDependentList<YoFrameOrientation>();
   private final SideDependentList<YoFrameOrientation> desiredFootOrientations = new SideDependentList<YoFrameOrientation>();

   private final YoFramePoint finalDesiredSwingFootPosition = new YoFramePoint("finalDesiredSwing", "", worldFrame, registry);
   private final YoFramePoint desiredSwingFootPositionInWorldFrame = new YoFramePoint("desiredSwing", "", worldFrame, registry);
   private final YoFrameVector desiredSwingFootVelocityInWorldFrame = new YoFrameVector("desiredSwingVelocity", "", worldFrame, registry);
   private final YoFrameVector desiredSwingFootAccelerationInWorldFrame = new YoFrameVector("desiredSwingAcceleration", "", worldFrame, registry);
   private final YoFrameVector desiredSwingFootAngularVelocityInWorldFrame = new YoFrameVector("desiredSwingAngularVelocity", "", worldFrame, registry);
   private final YoFrameVector desiredSwingFootAngularAccelerationInWorldFrame = new YoFrameVector("desiredSwingAngularAcceleration", "", worldFrame, registry);
   private YoGraphicCoordinateSystem swingFootOrientationViz = null, finalDesiredSwingOrientationViz = null;

   private final DoubleYoVariable positionErrorAtEndOfStepNorm = new DoubleYoVariable("positionErrorAtEndOfStepNorm", registry);
   private final DoubleYoVariable positionErrorAtEndOfStepX = new DoubleYoVariable("positionErrorAtEndOfStepX", registry);
   private final DoubleYoVariable positionErrorAtEndOfStepY = new DoubleYoVariable("positionErrorAtEndOfStepY", registry);

   private final YoFrameVector positionInSupportLegAnkleZUp = new YoFrameVector("positionInSupportLegAnkleZUp", ReferenceFrame.getWorldFrame(), registry);

   private final OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();

   private BagOfBalls bagOfBalls;
   private final double controlDT;
   private RobotSide swingSide;

   public ChangingEndpointSwingSubController(ProcessedSensorsInterface processedSensors, CommonHumanoidReferenceFrames referenceFrames,
         CouplingRegistry couplingRegistry, DesiredFootstepCalculator desiredFootstepCalculator,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry,
         SideDependentList<AnkleVelocityCalculator> ankleVelocityCalculators, SideDependentList<FootSwitchInterface> footSwitches,
         CartesianTrajectoryGenerator walkingCartesianTrajectoryGenerator,
         SideDependentList<CartesianTrajectoryGenerator> swingInAirCartesianTrajectoryGenerators, PreSwingControlModule preSwingControlModule,
         double controlDT, SwingLegTorqueControlModule swingLegTorqueControlModule)
   {
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      this.couplingRegistry = couplingRegistry;
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      this.swingLegTorqueControlModule = swingLegTorqueControlModule;
      this.ankleVelocityCalculators = new SideDependentList<AnkleVelocityCalculator>(ankleVelocityCalculators);
      this.footSwitches = new SideDependentList<FootSwitchInterface>(footSwitches);
      this.walkingTrajectoryGenerator = walkingCartesianTrajectoryGenerator;
      this.swingInAirTrajectoryGenerators = swingInAirCartesianTrajectoryGenerators;
      this.preSwingControlModule = preSwingControlModule;
      this.controlDT = controlDT;

      for (RobotSide side : RobotSide.values)
      {
         ReferenceFrame orientationReferenceFrame = referenceFrames.getAnkleZUpFrame(side.getOppositeSide());
         YoFrameOrientation startSwingOrientation = new YoFrameOrientation(side.getCamelCaseNameForStartOfExpression() + "startSwing", "",
               orientationReferenceFrame, registry);
         YoFrameOrientation endSwingOrientation = new YoFrameOrientation(side.getCamelCaseNameForStartOfExpression() + "endSwing", "",
               orientationReferenceFrame, registry);
         YoFrameOrientation desiredFootOrientation = new YoFrameOrientation(side.getCamelCaseNameForStartOfExpression() + "desiredSwing", "",
               orientationReferenceFrame, registry);
         startSwingOrientations.set(side, startSwingOrientation);
         endSwingOrientations.set(side, endSwingOrientation);
         desiredFootOrientations.set(side, desiredFootOrientation);
      }

      createVisualizers(yoGraphicsListRegistry, parentRegistry);
      couplingRegistry.setEstimatedSwingTimeRemaining(estimatedSwingTimeRemaining.getDoubleValue());
      parentRegistry.addChild(registry);
   }

   private void createVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      if (yoGraphicsListRegistry != null)
      {
         ArtifactList artifactList = new ArtifactList("ChangingEndpoint");

         swingFootOrientationViz = new YoGraphicCoordinateSystem("Coordinate System", desiredSwingFootPositionInWorldFrame,
               desiredFootOrientationInWorldFrame, 0.1);
         finalDesiredSwingOrientationViz = new YoGraphicCoordinateSystem("Final Desired Orientation", finalDesiredSwingFootPosition,
               finalDesiredFootOrientationInWorldFrame, 0.1);

         int numberOfBalls = 1;
         double ballSize = (numberOfBalls > 1) ? 0.005 : 0.02;
         bagOfBalls = new BagOfBalls(numberOfBalls, ballSize, "swingTarget", YoAppearance.Aqua(), parentRegistry, yoGraphicsListRegistry);

         YoGraphicPosition finalDesiredSwingViz = new YoGraphicPosition("Final Desired Swing", finalDesiredSwingFootPosition, 0.03,
               YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);

         yoGraphicsListRegistry.registerYoGraphics("R2Sim02SwingSubController", new YoGraphic[] { swingFootOrientationViz,
               finalDesiredSwingViz, finalDesiredSwingOrientationViz });

         artifactList.add(finalDesiredSwingViz.createArtifact());
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }
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

      return swingInAirTrajectoryGenerators.get(swingSide).isDone() && (deltaFootHeight < maxFootHeight);
   }

   private void updateGroundClearance(RobotSide robotSide)
   {
      FramePoint swingFootPoint = new FramePoint(referenceFrames.getAnkleZUpFrame(robotSide));
      swingFootPoint.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide.getOppositeSide()));

      positionInSupportLegAnkleZUp.set(swingFootPoint.getVectorCopy());
   }

   public void doPreSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(couplingRegistry.getSingleSupportDuration());
      this.swingSide = legTorquesToPackForSwingLeg.getRobotSide();
      preSwingControlModule.doPreSwing(legTorquesToPackForSwingLeg, timeInState);
      swingLegTorqueControlModule.computePreSwing(swingSide);
      timeSpentInPreSwing.set(timeInState);
      updateGroundClearance(swingSide);
   }

   public void doInitialSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      doInitialAndMidSwing(legTorquesToPackForSwingLeg, timeInState);
      timeSpentInInitialSwing.set(timeInState);
   }

   public void doMidSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      double timeSpentSwingingUpToNow = timeInState + timeSpentInInitialSwing.getDoubleValue();
      doInitialAndMidSwing(legTorquesToPackForSwingLeg, timeSpentSwingingUpToNow);
      timeSpentInMidSwing.set(timeInState);
   }

   private void doInitialAndMidSwing(LegTorques legTorquesToPackForSwingLeg, double timeSpentSwingingUpToNow)
   {
      this.swingSide = legTorquesToPackForSwingLeg.getRobotSide();
      updateFinalDesiredPosition(walkingTrajectoryGenerator);
      computeDesiredFootPosVelAcc(swingSide, walkingTrajectoryGenerator, timeSpentSwingingUpToNow);
      computeSwingLegTorques(legTorquesToPackForSwingLeg);
      setEstimatedSwingTimeRemaining(couplingRegistry.getSingleSupportDuration() - timeSpentSwingingUpToNow);
      updateGroundClearance(swingSide);
   }

   public void doTerminalSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(0.0);

      // Continue swinging to the same place in world coordinates, not the
      // same place in body coordinates...
      desiredSwingFootVelocityInWorldFrame.set(0.0, 0.0, 0.0);
      desiredSwingFootAngularVelocityInWorldFrame.set(0.0, 0.0, 0.0);

      desiredSwingFootAccelerationInWorldFrame.set(0.0, 0.0, 0.0);
      desiredSwingFootAngularAccelerationInWorldFrame.set(0.0, 0.0, 0.0);

      computeSwingLegTorques(legTorquesToPackForSwingLeg);

      updateGroundClearance(legTorquesToPackForSwingLeg.getRobotSide());

      timeSpentInTerminalSwing.set(timeInState);
   }

   public void doSwingInAir(LegTorques legTorques, double timeInCurrentState)
   {
      this.swingSide = legTorques.getRobotSide();

      FramePoint swingFootPosition = new FramePoint(referenceFrames.getFootFrame(swingSide));
      swingFootPosition.changeFrame(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
      double footZ = swingFootPosition.getZ();

      double minFootZ = 0.02;
      if (footZ < minFootZ)
         swingLegTorqueControlModule.setAnkleGainsSoft(swingSide);
      else
         swingLegTorqueControlModule.setAnkleGainsDefault(swingSide);

      computeDesiredFootPosVelAcc(swingSide, swingInAirTrajectoryGenerators.get(swingSide), timeInCurrentState);
      computeSwingLegTorques(legTorques);
   }

   public void doTransitionIntoPreSwing(RobotSide swingSide)
   {
      desiredFootstepCalculator.initializeDesiredFootstep(swingSide.getOppositeSide());

      // Reset the timers
      timeSpentInPreSwing.set(0.0);
      timeSpentInInitialSwing.set(0.0);
      timeSpentInMidSwing.set(0.0);
      timeSpentInTerminalSwing.set(0.0);
   }

   public void doTransitionIntoInitialSwing(RobotSide swingSide)
   {
      ReferenceFrame cartesianTrajectoryGeneratorFrame = walkingTrajectoryGenerator.getReferenceFrame();

      // Get the current position of the swing foot
      FramePoint startPoint = new FramePoint(referenceFrames.getAnkleZUpFrame(swingSide));

      // Get the desired position of the swing foot
      Footstep desiredFootstep = couplingRegistry.getDesiredFootstep();
      FramePoint endPoint = new FramePoint();
      desiredFootstep.getPositionIncludingFrame(endPoint);

      // Get the initial velocity of the swing foot
      FrameVector initialSwingVelocityVector = ankleVelocityCalculators.get(swingSide).getAnkleVelocityInWorldFrame();

      // Express everything in the same frame and initialize the trajectory generator
      startPoint.changeFrame(cartesianTrajectoryGeneratorFrame);
      endPoint.changeFrame(cartesianTrajectoryGeneratorFrame);
      initialSwingVelocityVector.changeFrame(cartesianTrajectoryGeneratorFrame);
      walkingTrajectoryGenerator.initialize(startPoint, initialSwingVelocityVector, null, endPoint, null);

      // Setup the orientation trajectory
      setupSwingFootOrientationTrajectory(desiredFootstep, swingSide);

      // Set the finalDesiredSwingPosition
      endPoint.changeFrame(finalDesiredSwingFootPosition.getReferenceFrame());
      finalDesiredSwingFootPosition.set(endPoint);

      FrameOrientation desiredFootstepOrientation = new FrameOrientation();
      desiredFootstep.getOrientationIncludingFrame(desiredFootstepOrientation);
      desiredFootstepOrientation.changeFrame(worldFrame);
      this.finalDesiredFootOrientationInWorldFrame.set(desiredFootstepOrientation);

      swingLegTorqueControlModule.setAnkleGainsDefault(swingSide);
      footSwitches.get(swingSide).reset();
   }

   public void doTransitionIntoMidSwing(RobotSide swingSide)
   {
   }

   public void doTransitionIntoTerminalSwing(RobotSide swingSide)
   {
      swingLegTorqueControlModule.setAnkleGainsSoft(swingSide);
   }

   public void doTransitionIntoSwingInAir(RobotSide swingLeg, BalanceOnOneLegConfiguration currentConfiguration)
   {

      minimumJerkTrajectoryForFootOrientation.setParams(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, swingOrientationTime.getDoubleValue());
      setEstimatedSwingTimeRemaining(0.0);

      FramePoint currentPosition = new FramePoint(referenceFrames.getFootFrame(swingLeg));

      FrameVector currentVelocity = ankleVelocityCalculators.get(swingLeg).getAnkleVelocityInWorldFrame();

      FramePoint finalDesiredPosition = currentConfiguration.getDesiredSwingFootPosition();

      swingInAirTrajectoryGenerators.get(swingLeg).initialize(currentPosition, currentVelocity, null, finalDesiredPosition, null);
   }

   public void doTransitionOutOfInitialSwing(RobotSide swingSide)
   {
      swingLegTorqueControlModule.setAnkleGainsDefault(swingSide);
   }

   public void doTransitionOutOfMidSwing(RobotSide swingSide)
   {
   }

   public void doTransitionOutOfPreSwing(RobotSide swingSide)
   {
   }

   public void doTransitionOutOfTerminalSwing(RobotSide swingSide)
   {
      updatePositionError(swingSide);
   }

   public void doTransitionOutOfSwingInAir(RobotSide swingLeg)
   {
      RobotSide supportLeg = swingLeg.getOppositeSide();
      desiredFootstepCalculator.initializeDesiredFootstep(supportLeg);
      // TODO: sort of nasty, but otherwise the swing trajectory won't be initialized correctly in doTransitionIntoInitialSwing:
      couplingRegistry.setDesiredFootstep(desiredFootstepCalculator.updateAndGetDesiredFootstep(swingLeg.getOppositeSide()));
      updatePositionError(swingSide);
   }

   private void setEstimatedSwingTimeRemaining(double timeRemaining)
   {
      this.estimatedSwingTimeRemaining.set(timeRemaining);
      this.couplingRegistry.setEstimatedSwingTimeRemaining(timeRemaining);
   }

   public double getEstimatedSwingTimeRemaining()
   {
      return estimatedSwingTimeRemaining.getDoubleValue();
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
      FramePoint2d sweetSpot = couplingRegistry.getOldBipedSupportPolygons().getSweetSpotCopy(oppositeSide);
      sweetSpot.changeFrame(stanceAnkleZUpFrame);
      boolean inStateLongEnough = timeInState > 0.05;
      boolean isCoMPastSweetSpot = comProjection.getX() > sweetSpot.getX();
      boolean trajectoryIsDone = walkingTrajectoryGenerator.isDone();
      boolean footHitEarly = footSwitches.get(swingSide).hasFootHitGround();

      return inStateLongEnough && (isCoMPastSweetSpot || trajectoryIsDone || footHitEarly);
   }

   public boolean isDoneWithMidSwing(RobotSide swingSide, double timeInState)
   {
      boolean trajectoryIsDone = walkingTrajectoryGenerator.isDone();

      return trajectoryIsDone;
   }

   public boolean isDoneWithTerminalSwing(RobotSide swingSide, double timeInState)
   {
      boolean footOnGround = footSwitches.get(swingSide).hasFootHitGround();

      boolean minimumTerminalSwingTimePassed = (timeInState > minimumTerminalSwingDuration.getDoubleValue());
      boolean maximumTerminalSwingTimePassed = (timeInState > maximumTerminalSwingDuration.getDoubleValue());

      boolean capturePointInsideFoot = isCapturePointInsideFoot(swingSide);

      return ((footOnGround && minimumTerminalSwingTimePassed) || maximumTerminalSwingTimePassed || (capturePointInsideFoot && minimumTerminalSwingTimePassed));
   }

   public boolean isDoneWithSwingInAir(RobotSide swingSide, double timeInState)
   {
      return swingInAirTrajectoryGenerators.get(swingSide).isDone() && (timeInState > 2.0);
   }

   public void setParametersForR2()
   {
      //      swingDuration.set(0.4);    // (0.4);
      swingOrientationTime.set(0.2); // 0.75 * swingDuration.getDoubleValue());

      swingToePitchUpOnLanding.set(0.25); // 0.4); // (0.5);

      initialSwingVelocity.set(0.2); // 0.12;
      initialSwingAcceleration.set(0.0);

      finalSwingVelocity.set(0.2); // 0.12;
      finalSwingAcceleration.set(0.0);

      minimumTerminalSwingDuration.set(0.03); // 0.1); // 0.25;
      maximumTerminalSwingDuration.set(0.15); // 0.15);    // 0.1); // 0.25;
      terminalSwingGainRampTime.set(minimumTerminalSwingDuration.getDoubleValue() / 4.0);

      passiveHipCollapseTime.set(0.07); // 0.06); // 0.1);

      antiGravityPercentage.set(1.0);

      comXThresholdToFinishInitialSwing.set(0.15);
   }

   public void setParametersForM2V2()
   {
      //      swingDuration.set(0.6);    // 0.7);    // 0.5);    // (0.4);
      swingOrientationTime.set(0.2); // 0.75 * swingDuration.getDoubleValue());

      swingToePitchUpOnLanding.set(0.25); // 0.4);    // (0.5);

      initialSwingVelocity.set(0.2); // 0.12;
      initialSwingAcceleration.set(0.0);

      finalSwingVelocity.set(0.2); // 0.12;
      finalSwingAcceleration.set(0.0);

      minimumTerminalSwingDuration.set(0.0); // 0.1);    // 0.25;
      maximumTerminalSwingDuration.set(0.05); // 0.2);    // 0.1);    // 0.25;
      terminalSwingGainRampTime.set(minimumTerminalSwingDuration.getDoubleValue() / 4.0);

      passiveHipCollapseTime.set(0.07); // 0.1);    // 07);    // 0.06);    // 0.1);

      antiGravityPercentage.set(1.0);

      comXThresholdToFinishInitialSwing.set(0.1); // 15);
   }

   private void updateFinalDesiredPosition(CartesianTrajectoryGenerator trajectoryGenerator)
   {
      Footstep desiredFootstep = couplingRegistry.getDesiredFootstep();

      FramePose desiredFootstepPose = new FramePose();
      desiredFootstep.getPose(desiredFootstepPose);
      FramePoint finalDesiredSwingFootPosition = new FramePoint();
      desiredFootstepPose.getPositionIncludingFrame(finalDesiredSwingFootPosition);
      finalDesiredSwingFootPosition.changeFrame(this.finalDesiredSwingFootPosition.getReferenceFrame());
      this.finalDesiredSwingFootPosition.set(finalDesiredSwingFootPosition);

      FrameOrientation desiredFootstepOrientation = new FrameOrientation();
      desiredFootstep.getOrientationIncludingFrame(desiredFootstepOrientation);
      desiredFootstepOrientation.changeFrame(worldFrame);
      this.finalDesiredFootOrientationInWorldFrame.set(desiredFootstepOrientation);

      ReferenceFrame cartesianTrajectoryGeneratorFrame = trajectoryGenerator.getReferenceFrame();
      finalDesiredSwingFootPosition.changeFrame(cartesianTrajectoryGeneratorFrame);
      trajectoryGenerator.updateFinalDesiredPosition(finalDesiredSwingFootPosition);
   }

   private void computeDesiredFootPosVelAcc(RobotSide swingSide, CartesianTrajectoryGenerator trajectoryGenerator, double timeInState)
   {
      ReferenceFrame cartesianTrajectoryGeneratorFrame = trajectoryGenerator.getReferenceFrame();

      // TODO: Don't generate so much junk here.
      FramePoint position = new FramePoint(cartesianTrajectoryGeneratorFrame);
      FrameVector velocity = new FrameVector(cartesianTrajectoryGeneratorFrame);
      FrameVector acceleration = new FrameVector(cartesianTrajectoryGeneratorFrame);

      trajectoryGenerator.computeNextTick(position, velocity, acceleration, controlDT);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      position.changeFrame(worldFrame);
      velocity.changeFrame(worldFrame);
      acceleration.changeFrame(worldFrame);

      desiredSwingFootPositionInWorldFrame.set(position);
      desiredSwingFootVelocityInWorldFrame.set(velocity);
      desiredSwingFootAccelerationInWorldFrame.set(acceleration);

      // Determine foot orientation and angular velocity
      minimumJerkTrajectoryForFootOrientation.computeTrajectory(timeInState);
      double orientationInterpolationAlpha = minimumJerkTrajectoryForFootOrientation.getPosition();
      YoFrameOrientation endSwingOrientation = endSwingOrientations.get(swingSide);
      YoFrameOrientation desiredFootOrientation = desiredFootOrientations.get(swingSide);
      YoFrameOrientation startSwingOrientation = startSwingOrientations.get(swingSide);
      desiredFootOrientation.interpolate(startSwingOrientation, endSwingOrientation, orientationInterpolationAlpha);

      // Visualisation
      FrameOrientation desiredFootOrientationToViz = desiredFootOrientation.getFrameOrientationCopy();
      desiredFootOrientationToViz.changeFrame(worldFrame);
      desiredFootOrientationInWorldFrame.set(desiredFootOrientationToViz);

      double alphaDot = minimumJerkTrajectoryForFootOrientation.getVelocity();
      FrameVector desiredSwingFootAngularVelocity = new FrameVector();
      orientationInterpolationCalculator.computeAngularVelocity(desiredSwingFootAngularVelocity, startSwingOrientation.getFrameOrientationCopy(),
            endSwingOrientation.getFrameOrientationCopy(), alphaDot);
      desiredSwingFootAngularVelocity.changeFrame(worldFrame);
      desiredSwingFootAngularVelocityInWorldFrame.set(desiredSwingFootAngularVelocity);

      double alphaDDot = minimumJerkTrajectoryForFootOrientation.getAcceleration();
      FrameVector desiredSwingFootAngularAcceleration = new FrameVector();
      orientationInterpolationCalculator.computeAngularAcceleration(desiredSwingFootAngularAcceleration, startSwingOrientation.getFrameOrientationCopy(),
            endSwingOrientation.getFrameOrientationCopy(), alphaDDot);
      desiredSwingFootAngularAcceleration.changeFrame(worldFrame);
      desiredSwingFootAngularAccelerationInWorldFrame.set(desiredSwingFootAngularAcceleration);

      updateSwingfootError(position);
   }

   private void updateSwingfootError(FramePoint desiredPosition)
   {
      ReferenceFrame swingFootFrame = referenceFrames.getFootFrame(swingSide);
      desiredPosition.changeFrame(swingFootFrame);
      swingFootPositionError.set(desiredPosition.distance(new FramePoint(swingFootFrame)));
   }

   private void computeSwingLegTorques(LegTorques legTorquesToPackForSwingLeg)
   {
      swingLegTorqueControlModule.compute(legTorquesToPackForSwingLeg, desiredSwingFootPositionInWorldFrame.getFramePointCopy(),
            desiredFootOrientations.get(legTorquesToPackForSwingLeg.getRobotSide()).getFrameOrientationCopy(),
            desiredSwingFootVelocityInWorldFrame.getFrameVectorCopy(), desiredSwingFootAngularVelocityInWorldFrame.getFrameVectorCopy(),
            desiredSwingFootAccelerationInWorldFrame.getFrameVectorCopy(), desiredSwingFootAngularAccelerationInWorldFrame.getFrameVectorCopy());

      leaveTrailOfBalls();
   }

   private void leaveTrailOfBalls()
   {
      if (bagOfBalls != null)
      {
         bagOfBalls.setBallLoop(desiredSwingFootPositionInWorldFrame.getFramePointCopy());
      }
   }

   private void setupSwingFootOrientationTrajectory(Footstep desiredFootStep, RobotSide swingSide)
   {
      minimumJerkTrajectoryForFootOrientation.setParams(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, swingOrientationTime.getDoubleValue());

      initializeStartOrientationToMatchActual(swingSide);

      FrameOrientation endOrientation = new FrameOrientation();
      desiredFootStep.getOrientationIncludingFrame(endOrientation);
      ReferenceFrame supportFootAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide());
      endOrientation.changeFrame(supportFootAnkleZUpFrame);
      endSwingOrientations.get(swingSide).set(endOrientation);
   }

   private void initializeStartOrientationToMatchActual(RobotSide swingSide)
   {
      ReferenceFrame swingFootFrame = referenceFrames.getFootFrame(swingSide);
      FrameOrientation startOrientation = new FrameOrientation(swingFootFrame);
      startOrientation.changeFrame(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
      startSwingOrientations.get(swingSide).set(startOrientation);
   }

   private boolean isCapturePointInsideFoot(RobotSide swingSide)
   {
      FrameConvexPolygon2d footPolygon = couplingRegistry.getOldBipedSupportPolygons().getFootPolygonInAnkleZUp(swingSide);
      FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(footPolygon.getReferenceFrame()).toFramePoint2d();

      boolean capturePointInsideFoot = footPolygon.isPointInside(capturePoint);

      return capturePointInsideFoot;
   }

   private void updatePositionError(RobotSide swingSide)
   {
      FramePoint currentPosition = new FramePoint(referenceFrames.getAnkleZUpFrame(swingSide));
      currentPosition.changeFrame(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
      FramePoint desiredPosition = new FramePoint(finalDesiredSwingFootPosition.getFramePointCopy());
      desiredPosition.changeFrame(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
      positionErrorAtEndOfStepNorm.set(desiredPosition.distance(currentPosition));
      currentPosition.sub(desiredPosition);
      positionErrorAtEndOfStepX.set(currentPosition.getX());
      positionErrorAtEndOfStepY.set(currentPosition.getY());
   }

   public void initialize()
   {
   }

   public void doPreSwingInAir(LegTorques legTorques, double timeInState)
   {
   }

   public void doTransitionIntoPreSwingInAir(RobotSide swingSide)
   {
   }

   public void doTransitionOutOfPreSwingInAir(RobotSide swingLeg)
   {
   }

   public boolean isDoneWithPreSwingInAir(RobotSide swingSide, double timeInState)
   {
      return true;
   }
}