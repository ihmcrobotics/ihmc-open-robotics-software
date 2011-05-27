package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.configurations.BalanceOnOneLegConfiguration;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PreSwingControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.LegJointPositionControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.kinematics.AnkleVelocityCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.DesiredJointAccelerationCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.DesiredJointVelocityCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.InverseKinematicsException;
import us.ihmc.commonWalkingControlModules.kinematics.LegInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.commonWalkingControlModules.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.utilities.kinematics.OrientationInterpolationCalculator;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.ArtifactList;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCoordinateSystem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.YoMinimumJerkTrajectory;

public class ChangingEndpointSwingSubController implements SwingSubController
{
   private final ProcessedSensorsInterface processedSensors;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final FullRobotModel fullRobotModel;
   private final CouplingRegistry couplingRegistry;

   private final DesiredFootstepCalculator desiredFootstepCalculator;

   private final CartesianTrajectoryGenerator walkingTrajectoryGenerator;
   private final CartesianTrajectoryGenerator swingInAirTrajectoryGenerator;

   private final SideDependentList<LegJointPositionControlModule> legJointPositionControlModules;
   private final SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators;

   private final LegInverseKinematicsCalculator inverseKinematicsCalculator;

   private final SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators;
   private final SideDependentList<DesiredJointAccelerationCalculator> desiredJointAccelerationCalculators;

   private final SideDependentList<AnkleVelocityCalculator> ankleVelocityCalculators;
   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final PreSwingControlModule preSwingControlModule;

   private final YoVariableRegistry registry = new YoVariableRegistry("SwingSubConroller");

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final BooleanYoVariable inverseKinematicsExceptionHasBeenThrown = new BooleanYoVariable("kinematicException", registry);
   private final DoubleYoVariable jacobianDeterminant = new DoubleYoVariable("jacobianDeterminant", registry);

   private final DoubleYoVariable passiveHipCollapseTime = new DoubleYoVariable("passiveHipCollapseTime", registry);

   private final DoubleYoVariable swingDuration = new DoubleYoVariable("swingDuration", "The duration of the swing movement. [s]", registry);
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

   private final DoubleYoVariable comXThresholdToFinishInitialSwing =
      new DoubleYoVariable("comXThresholdToFinishInitialSwing",
                           "How far the CoM should be in front of the support foot before transitioning out of initial swing.", registry);

   private final DoubleYoVariable timeSpentInPreSwing = new DoubleYoVariable("timeSpentInPreSwing", "This is the time spent in Pre swing.", registry);
   private final DoubleYoVariable timeSpentInInitialSwing = new DoubleYoVariable("timeSpentInInitialSwing", "This is the time spent in initial swing.",
                                                               registry);
   private final DoubleYoVariable timeSpentInMidSwing = new DoubleYoVariable("timeSpentInMidSwing", "This is the time spend in mid swing.", registry);
   private final DoubleYoVariable timeSpentInTerminalSwing = new DoubleYoVariable("timeSpentInTerminalSwing", "This is the time spent in terminal swing.",
                                                                registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", "This is the toal time spent in single support.",
                                                             registry);

   private final DoubleYoVariable swingFootPositionError = new DoubleYoVariable("swingFootPositionError", registry);

   private final YoMinimumJerkTrajectory minimumJerkTrajectoryForFootOrientation = new YoMinimumJerkTrajectory("swingFootOrientation", registry);

   private final YoFrameOrientation startSwingOrientation = new YoFrameOrientation("startSwing", "", worldFrame, registry);
   private final YoFrameOrientation endSwingOrientation = new YoFrameOrientation("endSwing", "", worldFrame, registry);
   private final YoFrameOrientation desiredFootOrientation = new YoFrameOrientation("desiredSwing", "", worldFrame, registry);

   private final YoFramePoint finalDesiredSwingFootPosition = new YoFramePoint("finalDesiredSwing", "", worldFrame, registry);
   private final YoFramePoint desiredSwingFootPositionInWorldFrame = new YoFramePoint("desiredSwing", "", worldFrame, registry);
   private final YoFrameVector desiredSwingFootVelocityInWorldFrame = new YoFrameVector("desiredSwingVelocity", "", worldFrame, registry);
   private final YoFrameVector desiredSwingFootAccelerationInWorldFrame = new YoFrameVector("desiredSwingAcceleration", "", worldFrame, registry);
   private final YoFrameVector desiredSwingFootAngularVelocityInWorldFrame = new YoFrameVector("desiredSwingAngularVelocity", "", worldFrame, registry);
   private final YoFrameVector desiredSwingFootAngularAccelerationInWorldFrame = new YoFrameVector("desiredSwingAngularAcceleration", "", worldFrame, registry);
   private DynamicGraphicCoordinateSystem swingFootOrientationViz = null;

   private LegJointPositions desiredLegJointPositions;
   private LegJointVelocities desiredLegJointVelocities;

   private BagOfBalls bagOfBalls;
   private final double controlDT;
   private boolean useBodyAcceleration;
   private RobotSide swingSide;

   public ChangingEndpointSwingSubController(ProcessedSensorsInterface processedSensors, CommonWalkingReferenceFrames referenceFrames,
           FullRobotModel fullRobotModel, CouplingRegistry couplingRegistry, DesiredFootstepCalculator desiredFootstepCalculator,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry,
           SideDependentList<LegJointPositionControlModule> legJointPositionControlModules,
           SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators,
           SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators,
           SideDependentList<DesiredJointAccelerationCalculator> desiredJointAccelerationCalculators,
           LegInverseKinematicsCalculator inverseKinematicsCalculator, SideDependentList<AnkleVelocityCalculator> ankleVelocityCalculators,
           SideDependentList<FootSwitchInterface> footSwitches, CartesianTrajectoryGenerator walkingCartesianTrajectoryGenerator,
           CartesianTrajectoryGenerator swingInAirCartesianTrajectoryGenerator, PreSwingControlModule preSwingControlModule, double controlDT)
   {
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      this.fullRobotModel = fullRobotModel;
      this.couplingRegistry = couplingRegistry;
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      this.legJointPositionControlModules = new SideDependentList<LegJointPositionControlModule>(legJointPositionControlModules);
      this.inverseDynamicsCalculators = new SideDependentList<InverseDynamicsCalculator>(inverseDynamicsCalculators);
      this.inverseKinematicsCalculator = inverseKinematicsCalculator;
      this.desiredJointVelocityCalculators = new SideDependentList<DesiredJointVelocityCalculator>(desiredJointVelocityCalculators);
      this.desiredJointAccelerationCalculators = new SideDependentList<DesiredJointAccelerationCalculator>(desiredJointAccelerationCalculators);
      this.ankleVelocityCalculators = new SideDependentList<AnkleVelocityCalculator>(ankleVelocityCalculators);
      this.footSwitches = new SideDependentList<FootSwitchInterface>(footSwitches);
      this.walkingTrajectoryGenerator = walkingCartesianTrajectoryGenerator;
      this.swingInAirTrajectoryGenerator = swingInAirCartesianTrajectoryGenerator;
      this.preSwingControlModule = preSwingControlModule;
      this.controlDT = controlDT;

      createVisualizers(dynamicGraphicObjectsListRegistry, parentRegistry);
      couplingRegistry.setEstimatedSwingTimeRemaining(estimatedSwingTimeRemaining.getDoubleValue());
      couplingRegistry.setSingleSupportDuration(swingDuration.getDoubleValue());
      parentRegistry.addChild(registry);
   }

   private void createVisualizers(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      if (dynamicGraphicObjectsListRegistry != null)
      {
         ArtifactList artifactList = new ArtifactList("ChangingEndpoint");

         swingFootOrientationViz = new DynamicGraphicCoordinateSystem("Coordinate System", desiredSwingFootPositionInWorldFrame, desiredFootOrientation, 0.1);

         int numberOfBalls = 1;
         double ballSize = (numberOfBalls > 1) ? 0.005 : 0.02;
         bagOfBalls = new BagOfBalls(numberOfBalls, ballSize, "swingTarget", YoAppearance.Aqua(), parentRegistry, dynamicGraphicObjectsListRegistry);


         DynamicGraphicPosition finalDesiredSwingViz = finalDesiredSwingFootPosition.createDynamicGraphicPosition("Final Desired Swing", 0.03,
                                                          YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjects("R2Sim02SwingSubController", new DynamicGraphicObject[] {swingFootOrientationViz,
                 finalDesiredSwingViz});

         artifactList.add(finalDesiredSwingViz.createArtifact());
         dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
      }
   }

   public boolean canWeStopNow()
   {
      return true;
   }

   public boolean isReadyForDoubleSupport()
   {
      FramePoint swingAnkle = new FramePoint(referenceFrames.getFootFrame(swingSide));
      swingAnkle.changeFrame(referenceFrames.getAnkleZUpFrame(swingSide.getOppositeSide()));
      double footHeight = swingAnkle.getZ();
      double maxFootHeight = 0.02;

      return swingInAirTrajectoryGenerator.isDone() && (footHeight < maxFootHeight);
   }

   public void doPreSwing(LegTorques legTorquesToPackForSwingLeg, double timeInState)
   {
      setEstimatedSwingTimeRemaining(swingDuration.getDoubleValue());
      this.swingSide = legTorquesToPackForSwingLeg.getRobotSide();
      preSwingControlModule.doPreSwing(legTorquesToPackForSwingLeg, timeInState);

      fullRobotModel.getRootJoint().setDesiredAccelerationToZero();

      for (LegJointName legJointName : legTorquesToPackForSwingLeg.getLegJointNames())
      {
         fullRobotModel.getLegJoint(swingSide, legJointName).setQddDesired(0.0);
      }

      inverseDynamicsCalculators.get(swingSide).compute();
      setUpperBodyWrench();

      timeSpentInPreSwing.set(timeInState);
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
      computeDesiredFootPosVelAcc(walkingTrajectoryGenerator, timeSpentSwingingUpToNow);
      computeSwingLegTorques(legTorquesToPackForSwingLeg);
      setEstimatedSwingTimeRemaining(swingDuration.getDoubleValue() - timeSpentSwingingUpToNow);
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
         legJointPositionControlModules.get(swingSide).setAnkleGainsSoft();
      else
         legJointPositionControlModules.get(swingSide).resetScalesToDefault();

      computeDesiredFootPosVelAcc(swingInAirTrajectoryGenerator, timeInCurrentState);
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
      singleSupportDuration.set(0.0);
   }

   public void doTransitionIntoInitialSwing(RobotSide swingSide)
   {
      ReferenceFrame cartesianTrajectoryGeneratorFrame = walkingTrajectoryGenerator.getReferenceFrame();

      // Get the current position of the swing foot
      FramePoint startPoint = new FramePoint(referenceFrames.getAnkleZUpFrame(swingSide));

      // Get the desired position of the swing foot
      Footstep desiredFootstep = couplingRegistry.getDesiredFootstep();
      FramePoint endPoint = new FramePoint(desiredFootstep.getFootstepPose().getPosition());

      // Get the initial velocity of the swing foot
      FrameVector initialSwingVelocityVector = ankleVelocityCalculators.get(swingSide).getAnkleVelocityInWorldFrame();

      // Express everything in the same frame and initialize the trajectory generator
      startPoint.changeFrame(cartesianTrajectoryGeneratorFrame);
      endPoint.changeFrame(cartesianTrajectoryGeneratorFrame);
      initialSwingVelocityVector.changeFrame(cartesianTrajectoryGeneratorFrame);
      walkingTrajectoryGenerator.initialize(startPoint, initialSwingVelocityVector, endPoint);

      // Setup the orientation trajectory
      setupSwingFootOrientationTrajectory(desiredFootstep);

      // Set the finalDesiredSwingPosition
      endPoint.changeFrame(finalDesiredSwingFootPosition.getReferenceFrame());
      finalDesiredSwingFootPosition.set(endPoint);

      legJointPositionControlModules.get(swingSide).resetScalesToDefault();
   }

   public void doTransitionIntoMidSwing(RobotSide swingSide)
   {
   }

   public void doTransitionIntoTerminalSwing(RobotSide swingSide)
   {
      legJointPositionControlModules.get(swingSide).setAnkleGainsSoft();
   }

   public void doTransitionIntoSwingInAir(RobotSide swingLeg, BalanceOnOneLegConfiguration currentConfiguration)
   {
      setEstimatedSwingTimeRemaining(0.0);

      ReferenceFrame trajectoryGeneratorReferenceFrame = swingInAirTrajectoryGenerator.getReferenceFrame();

      FramePoint currentPosition = new FramePoint(referenceFrames.getFootFrame(swingLeg));
      currentPosition.changeFrame(trajectoryGeneratorReferenceFrame);

      FrameVector currentVelocity = ankleVelocityCalculators.get(swingLeg).getAnkleVelocityInWorldFrame();
      currentVelocity.changeFrame(trajectoryGeneratorReferenceFrame);

      FramePoint finalDesiredPosition = currentConfiguration.getDesiredSwingFootPosition();
      finalDesiredPosition.changeFrame(trajectoryGeneratorReferenceFrame);

      swingInAirTrajectoryGenerator.initialize(currentPosition, currentVelocity, finalDesiredPosition);
   }

   public void doTransitionOutOfInitialSwing(RobotSide swingSide)
   {
      legJointPositionControlModules.get(swingSide).resetScalesToDefault();
   }

   public void doTransitionOutOfMidSwing(RobotSide swingSide)
   {
   }

   public void doTransitionOutOfPreSwing(RobotSide swingSide)
   {
   }

   public void doTransitionOutOfTerminalSwing(RobotSide swingSide)
   {
      singleSupportDuration.set(timeSpentInPreSwing.getDoubleValue() + timeSpentInInitialSwing.getDoubleValue() + timeSpentInMidSwing.getDoubleValue()
                                + timeSpentInTerminalSwing.getDoubleValue());
      couplingRegistry.setSingleSupportDuration(singleSupportDuration.getDoubleValue());
   }

   public void doTransitionOutOfSwingInAir(RobotSide swingLeg)
   {
      // TODO Auto-generated method stub

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
      FramePoint2d sweetSpot = couplingRegistry.getBipedSupportPolygons().getSweetSpotCopy(oppositeSide);
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
      boolean capturePointInsideFoot = isCapturePointInsideFoot(swingSide);

      return trajectoryIsDone || capturePointInsideFoot;
   }

   public boolean isDoneWithTerminalSwing(RobotSide swingSide, double timeInState)
   {
      boolean footOnGround = footSwitches.get(swingSide).hasFootHitGround();

      boolean minimumTerminalSwingTimePassed = (timeInState > minimumTerminalSwingDuration.getDoubleValue());
      boolean maximumTerminalSwingTimePassed = (timeInState > maximumTerminalSwingDuration.getDoubleValue());

      boolean capturePointInsideFoot = isCapturePointInsideFoot(swingSide);

      return ((footOnGround && minimumTerminalSwingTimePassed) || maximumTerminalSwingTimePassed || (capturePointInsideFoot && minimumTerminalSwingTimePassed));
   }

   public boolean isDoneWithSwingInAir(double timeInState)
   {
      return swingInAirTrajectoryGenerator.isDone() && (timeInState > 2.0);
   }

   public void setParametersForR2()
   {
      swingDuration.set(0.4);    // (0.4);
      swingOrientationTime.set(0.2);    // 0.75 * swingDuration.getDoubleValue());

      swingToePitchUpOnLanding.set(0.25);    // 0.4); // (0.5);

      initialSwingVelocity.set(0.2);    // 0.12;
      initialSwingAcceleration.set(0.0);

      finalSwingVelocity.set(0.2);    // 0.12;
      finalSwingAcceleration.set(0.0);

      minimumTerminalSwingDuration.set(0.03);    // 0.1); // 0.25;
      maximumTerminalSwingDuration.set(0.15);    // 0.15);    // 0.1); // 0.25;
      terminalSwingGainRampTime.set(minimumTerminalSwingDuration.getDoubleValue() / 4.0);

      passiveHipCollapseTime.set(0.07);    // 0.06); // 0.1);

      antiGravityPercentage.set(1.0);

      comXThresholdToFinishInitialSwing.set(0.15);

      useBodyAcceleration = false;
   }

   public void setParametersForM2V2()
   {
      swingDuration.set(0.6);    // 0.7);    // 0.5);    // (0.4);
      swingOrientationTime.set(0.2);    // 0.75 * swingDuration.getDoubleValue());

      swingToePitchUpOnLanding.set(0.25);    // 0.4);    // (0.5);

      initialSwingVelocity.set(0.2);    // 0.12;
      initialSwingAcceleration.set(0.0);

      finalSwingVelocity.set(0.2);    // 0.12;
      finalSwingAcceleration.set(0.0);

      minimumTerminalSwingDuration.set(0.0);    // 0.1);    // 0.25;
      maximumTerminalSwingDuration.set(0.05);    // 0.2);    // 0.1);    // 0.25;
      terminalSwingGainRampTime.set(minimumTerminalSwingDuration.getDoubleValue() / 4.0);

      passiveHipCollapseTime.set(0.07);    // 0.1);    // 07);    // 0.06);    // 0.1);

      antiGravityPercentage.set(1.0);

      comXThresholdToFinishInitialSwing.set(0.10);    // 15);

      useBodyAcceleration = true;
   }

   private void updateFinalDesiredPosition(CartesianTrajectoryGenerator trajectoryGenerator)
   {
      Footstep desiredFootstep = couplingRegistry.getDesiredFootstep();
      FramePoint finalDesiredSwingFootPosition =
         desiredFootstep.getFootstepPose().getPosition().changeFrameCopy(this.finalDesiredSwingFootPosition.getReferenceFrame());
      this.finalDesiredSwingFootPosition.set(finalDesiredSwingFootPosition);

      ReferenceFrame cartesianTrajectoryGeneratorFrame = trajectoryGenerator.getReferenceFrame();
      finalDesiredSwingFootPosition.changeFrame(cartesianTrajectoryGeneratorFrame);
      trajectoryGenerator.updateFinalDesiredPosition(finalDesiredSwingFootPosition);
   }

   private void computeDesiredFootPosVelAcc(CartesianTrajectoryGenerator trajectoryGenerator, double timeInState)
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
      desiredFootOrientation.interpolate(startSwingOrientation, endSwingOrientation, orientationInterpolationAlpha);

      double alphaDot = minimumJerkTrajectoryForFootOrientation.getVelocity();
      FrameVector desiredSwingFootAngularVelocity = OrientationInterpolationCalculator.computeAngularVelocity(startSwingOrientation.getFrameOrientationCopy(),
                                                       endSwingOrientation.getFrameOrientationCopy(), alphaDot);
      desiredSwingFootAngularVelocityInWorldFrame.set(desiredSwingFootAngularVelocity);

      double alphaDDot = minimumJerkTrajectoryForFootOrientation.getAcceleration();
      FrameVector desiredSwingFootAngularAcceleration =
         OrientationInterpolationCalculator.computeAngularAcceleration(startSwingOrientation.getFrameOrientationCopy(),
            endSwingOrientation.getFrameOrientationCopy(), alphaDDot);
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
      // robotSides
      RobotSide swingSide = legTorquesToPackForSwingLeg.getRobotSide();

      // reference frames
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      ReferenceFrame footFrame = referenceFrames.getFootFrame(swingSide);
      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();

      // Desired positions
      Transform3D footToPelvis = computeDesiredTransform(pelvisFrame);
      Twist desiredTwistOfSwingFootWithRespectToWorld = computeDesiredTwist(worldFrame, footFrame);

      desiredLegJointPositions = new LegJointPositions(swingSide);    // TODO: don't create every tick.
      Matrix3d footToPelvisOrientation = new Matrix3d();
      footToPelvis.get(footToPelvisOrientation);
      double desiredHipYaw = RotationFunctions.getYaw(footToPelvisOrientation);    // TODO: wrong and not necessary for R2, but ok for now.
      try
      {
         inverseKinematicsCalculator.solve(desiredLegJointPositions, footToPelvis, swingSide, desiredHipYaw);
         inverseKinematicsExceptionHasBeenThrown.set(false);
      }
      catch (InverseKinematicsException e)
      {
         inverseKinematicsExceptionHasBeenThrown.set(true);
      }

      // Desired velocities
      DesiredJointVelocityCalculator desiredJointVelocityCalculator = desiredJointVelocityCalculators.get(swingSide);
      desiredLegJointVelocities = desiredJointVelocityCalculator.computeDesiredJointVelocities(desiredTwistOfSwingFootWithRespectToWorld);

      // set body acceleration
      if (useBodyAcceleration)
      {
         SpatialAccelerationVector bodyAcceleration = processedSensors.computeAccelerationOfPelvisWithRespectToWorld();    // FIXME: set to LIPM-based predicted body acceleration
         bodyAcceleration.setAngularPart(new Vector3d());    // zero desired angular acceleration
         bodyAcceleration.setLinearPart(new Vector3d());    // zero linear acceleration as well for now
         fullRobotModel.getRootJoint().setDesiredAcceleration(bodyAcceleration);
      }

      // Desired acceleration
      SpatialAccelerationVector desiredAccelerationOfSwingFootWithRespectToWorld = computeDesiredSwingFootSpatialAcceleration(elevatorFrame, footFrame);
      jacobianDeterminant.set(desiredJointVelocityCalculator.swingFullLegJacobianDeterminant());
      desiredJointAccelerationCalculators.get(swingSide).compute(desiredAccelerationOfSwingFootWithRespectToWorld);

      double percentScaling = getPercentScalingBasedOnJacobianDeterminant(jacobianDeterminant.getDoubleValue());

      LegJointName[] legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
      for (LegJointName legJointName : legJointNames)
      {
         // this is better than not using the torques from the ID calculator at all, because at least gravity and Coriolis forces are compensated for
         RevoluteJoint revoluteJoint = fullRobotModel.getLegJoint(swingSide, legJointName);
         double qddDesired = revoluteJoint.getQddDesired();
         revoluteJoint.setQddDesired(qddDesired * percentScaling);
      }

      // control
      legJointPositionControlModules.get(swingSide).packTorquesForLegJointsPositionControl(percentScaling, legTorquesToPackForSwingLeg,
                                         desiredLegJointPositions, desiredLegJointVelocities);

      inverseDynamicsCalculators.get(swingSide).compute();

      for (LegJointName legJointName : legTorquesToPackForSwingLeg.getLegJointNames())
      {
         double tauInverseDynamics = fullRobotModel.getLegJoint(swingSide, legJointName).getTau();
         legTorquesToPackForSwingLeg.addTorque(legJointName, tauInverseDynamics);
      }

      setUpperBodyWrench();

      leaveTrailOfBalls();
   }

   private void setUpperBodyWrench()
   {
      Wrench upperBodyWrench = new Wrench();
      fullRobotModel.getRootJoint().packWrench(upperBodyWrench);
      upperBodyWrench.changeBodyFrameAttachedToSameBody(referenceFrames.getPelvisFrame());
      upperBodyWrench.changeFrame(referenceFrames.getPelvisFrame());
      couplingRegistry.setUpperBodyWrench(upperBodyWrench);
   }

   private double getPercentScalingBasedOnJacobianDeterminant(double jacobianDeterminant)
   {
      double determinantThresholdOne = 0.06;    // 0.05;    // 0.025;
      double determinantThresholdTwo = 0.03;    // 0.02; //0.01;

      double percent = (Math.abs(jacobianDeterminant) - determinantThresholdTwo) / (determinantThresholdOne - determinantThresholdTwo);
      percent = MathTools.clipToMinMax(percent, 0.0, 1.0);

      return percent;
   }

   private Transform3D computeDesiredTransform(ReferenceFrame pelvisFrame)
   {
      Orientation desiredFootOrientationInPelvisFrame = desiredFootOrientation.getFrameOrientationCopy();
      desiredFootOrientationInPelvisFrame.changeFrame(pelvisFrame);
      FramePoint desiredSwingFootPositionInPelvisFrame = desiredSwingFootPositionInWorldFrame.getFramePointCopy();
      desiredSwingFootPositionInPelvisFrame.changeFrame(pelvisFrame);
      Transform3D footToPelvis = createTransform(desiredFootOrientationInPelvisFrame, desiredSwingFootPositionInPelvisFrame);

      return footToPelvis;
   }

   private Twist computeDesiredTwist(ReferenceFrame worldFrame, ReferenceFrame footFrame)
   {
      FrameVector desiredSwingFootVelocity = desiredSwingFootVelocityInWorldFrame.getFrameVectorCopy();
      desiredSwingFootVelocity.changeFrame(footFrame);
      FrameVector desiredSwingFootAngularVelocity = desiredSwingFootAngularVelocityInWorldFrame.getFrameVectorCopy();
      desiredSwingFootAngularVelocity.changeFrame(footFrame);
      Twist desiredTwistOfSwingFootWithRespectToStanceFoot = new Twist(footFrame, worldFrame, footFrame, desiredSwingFootVelocity.getVector(),
                                                                desiredSwingFootAngularVelocity.getVector());

      return desiredTwistOfSwingFootWithRespectToStanceFoot;
   }

   private static Transform3D createTransform(Orientation orientation, FramePoint framePoint)
   {
      orientation.checkReferenceFrameMatch(framePoint);
      Matrix3d rotationMatrix = orientation.getMatrix3d();
      Transform3D ret = new Transform3D(rotationMatrix, new Vector3d(framePoint.getPoint()), 1.0);

      return ret;
   }

   private SpatialAccelerationVector computeDesiredSwingFootSpatialAcceleration(ReferenceFrame elevatorFrame, ReferenceFrame footFrame)
   {
      FrameVector desiredSwingFootAcceleration = desiredSwingFootAccelerationInWorldFrame.getFrameVectorCopy();
      desiredSwingFootAcceleration = desiredSwingFootAcceleration.changeFrameCopy(footFrame);
      FrameVector desiredSwingFootAngularAcceleration = desiredSwingFootAngularAccelerationInWorldFrame.getFrameVectorCopy();
      desiredSwingFootAngularAcceleration = desiredSwingFootAngularAcceleration.changeFrameCopy(footFrame);
      SpatialAccelerationVector desiredAccelerationOfSwingFootWithRespectToWorld = new SpatialAccelerationVector(footFrame, elevatorFrame, footFrame,
                                                                                      desiredSwingFootAcceleration.getVector(),
                                                                                      desiredSwingFootAngularAcceleration.getVector());

      return desiredAccelerationOfSwingFootWithRespectToWorld;
   }

   private void leaveTrailOfBalls()
   {
      if (bagOfBalls != null)
      {
         bagOfBalls.setBallLoop(desiredSwingFootPositionInWorldFrame.getFramePointCopy());
      }
   }

   private void setupSwingFootOrientationTrajectory(Footstep desiredFootStep)
   {
      // Why do we have Xf = 1.0 ?
      minimumJerkTrajectoryForFootOrientation.setParams(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, swingOrientationTime.getDoubleValue());

      initializeStartOrientationToMatchActual(desiredFootStep.getFootstepSide());

//    Orientation endOrientation = desiredFootStep.getFootstepPose().getOrientation().changeFrameCopy(endSwingOrientation.getReferenceFrame());
//    endSwingOrientation.set(endOrientation);

      Orientation endOrientation = desiredFootStep.getFootstepPose().getOrientation();
      endSwingOrientation.set(endOrientation.changeFrameCopy(worldFrame));
   }

   private void initializeStartOrientationToMatchActual(RobotSide swingSide)
   {
      ReferenceFrame swingFootFrame = referenceFrames.getFootFrame(swingSide);
      Orientation startOrientation = new Orientation(swingFootFrame);
      startOrientation = startOrientation.changeFrameCopy(ReferenceFrame.getWorldFrame());
      startSwingOrientation.set(startOrientation);
   }

   private boolean isCapturePointInsideFoot(RobotSide swingSide)
   {
      FrameConvexPolygon2d footPolygon = couplingRegistry.getBipedSupportPolygons().getFootPolygonInAnkleZUp(swingSide);
      FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(footPolygon.getReferenceFrame()).toFramePoint2d();

      boolean capturePointInsideFoot = footPolygon.isPointInside(capturePoint);

      return capturePointInsideFoot;
   }
}
