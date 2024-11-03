package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters.SmoothFootUnloadMethod;
import us.ihmc.commonWalkingControlModules.configurations.YoSwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold.PartialFootholdModuleParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold.YoPartialFootholdModuleParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff.CentroidProjectionToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff.DynamicStateInspectorParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff.GeometricToeOffManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.heightPlanning.CoMHeightTimeDerivativesData;
import us.ihmc.commonWalkingControlModules.heightPlanning.CoMHeightTimeDerivativesDataBasics;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.LegTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FeetManager implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double extraCoMHeightWithToes = 0.06;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final BooleanParameter useWorldSurfaceNormalWhenFullyConstrained = new BooleanParameter("useWorldSurfaceNormalWhenFullyConstrained", registry, true);
   private final DoubleProvider extraCoMMaxHeightWithToes;

   private final SideDependentList<FootControlModule> footControlModules = new SideDependentList<>();

   private final ToeOffParameters toeOffParameters;
   private final ToeOffCalculator toeOffCalculator;
   private final GeometricToeOffManager toeOffManager;

   private final SideDependentList<ContactableFoot> feet;

   private final ReferenceFrame pelvisZUpFrame;
   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final FramePoint3D tempSolePosition = new FramePoint3D();
   private final DoubleParameter blindFootstepsHeightOffset;

   private final CoMHeightTimeDerivativesDataBasics leftLegCoMHeightData = new CoMHeightTimeDerivativesData();
   private final CoMHeightTimeDerivativesDataBasics rightLegCoMHeightData = new CoMHeightTimeDerivativesData();
   private final SideDependentList<CoMHeightTimeDerivativesDataBasics> legComHeightData = new SideDependentList<>(leftLegCoMHeightData, rightLegCoMHeightData);

   public FeetManager(HighLevelHumanoidControllerToolbox controllerToolbox,
                      WalkingControllerParameters walkingControllerParameters,
                      PIDSE3GainsReadOnly swingFootGains,
                      PIDSE3GainsReadOnly holdFootGains,
                      PIDSE3GainsReadOnly toeOffFootGains,
                      SideDependentList<RigidBodyControlManager> flamingoFootControlManagers,
                      YoRegistry parentRegistry,
                      YoGraphicsListRegistry graphicsListRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.toeOffParameters = walkingControllerParameters.getToeOffParameters();
      feet = controllerToolbox.getContactableFeet();

      extraCoMMaxHeightWithToes = new DoubleParameter("extraCoMMaxHeightWithToes", registry, extraCoMHeightWithToes);

      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
         contactStates.put(robotSide, controllerToolbox.getFootContactState(robotSide));

      toeOffCalculator = new CentroidProjectionToeOffCalculator(contactStates,
                                                                feet,
                                                                walkingControllerParameters.getToeOffParameters(),
                                                                registry,
                                                                graphicsListRegistry);

      toeOffManager = new GeometricToeOffManager(controllerToolbox,
                                                 walkingControllerParameters,
                                                 new DynamicStateInspectorParameters(registry),
                                                 toeOffCalculator,
                                                 registry);

      this.footSwitches = controllerToolbox.getFootSwitches();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      soleZUpFrames = referenceFrames.getSoleZUpFrames();

      ExplorationParameters explorationParameters = null;
      if (walkingControllerParameters.createFootholdExplorationTools())
      {
         explorationParameters = new ExplorationParameters(registry);
      }
      // FIXME pass these parameters in from the robot, rather than creating here.
      YoPartialFootholdModuleParameters footholdRotationParameters = new YoPartialFootholdModuleParameters(new PartialFootholdModuleParameters(), registry);
      SupportStateParameters supportStateParameters = new SupportStateParameters(walkingControllerParameters, registry);

      SmoothFootUnloadMethod smoothUnloading = walkingControllerParameters.enforceSmoothFootUnloading();
      DoubleProvider minWeightFractionPerFoot = null;
      DoubleProvider maxWeightFractionPerFoot = null;
      DoubleProvider unloadedFinalRhoWeight = null;

      if (smoothUnloading == SmoothFootUnloadMethod.HARD_CONSTRAINT)
      {
         minWeightFractionPerFoot = new DoubleParameter("minWeightFractionPerFoot", registry, 0.0);
         maxWeightFractionPerFoot = new DoubleParameter("maxWeightFractionPerFoot", registry, 2.0);
      }
      else if (smoothUnloading == SmoothFootUnloadMethod.RHO_WEIGHT)
      {
         unloadedFinalRhoWeight = new DoubleParameter("unloadedFinalRhoWeight", registry, walkingControllerParameters.getFinalUnloadedRhoWeight());
      }

      WorkspaceLimiterParameters workspaceLimiterParameters = new WorkspaceLimiterParameters(registry);
      YoSwingTrajectoryParameters swingTrajectoryParameters = new YoSwingTrajectoryParameters("FootSwing", walkingControllerParameters, registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = new FootControlModule(robotSide,
                                                                     toeOffCalculator,
                                                                     walkingControllerParameters,
                                                                     swingTrajectoryParameters,
                                                                     workspaceLimiterParameters,
                                                                     swingFootGains,
                                                                     holdFootGains,
                                                                     toeOffFootGains,
                                                                     flamingoFootControlManagers.get(robotSide),
                                                                     controllerToolbox,
                                                                     explorationParameters,
                                                                     footholdRotationParameters,
                                                                     supportStateParameters,
                                                                     minWeightFractionPerFoot,
                                                                     maxWeightFractionPerFoot,
                                                                     unloadedFinalRhoWeight,
                                                                     registry);

         footControlModules.put(robotSide, footControlModule);
      }

      double defaultBlindFootstepsHeightOffset = walkingControllerParameters.getSwingTrajectoryParameters().getBlindFootstepsHeightOffset();
      blindFootstepsHeightOffset = new DoubleParameter("blindFootstepsHeightOffset", registry, defaultBlindFootstepsHeightOffset);

      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3DReadOnly loadedFootAngularWeight,
                          Vector3DReadOnly loadedFootLinearWeight,
                          Vector3DReadOnly footAngularWeight,
                          Vector3DReadOnly footLinearWeight)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.setWeights(loadedFootAngularWeight, loadedFootLinearWeight, footAngularWeight, footLinearWeight);
      }
   }

   public void initialize()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).initialize();
      }
   }

   public void compute()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footSwitches.get(robotSide).hasFootHitGroundFiltered(); //debug
         footControlModules.get(robotSide).doControl();
      }
   }

   public boolean adjustHeightIfNeeded(Footstep footstep)
   {
      if (!footstep.getTrustHeight())
      {
         tempSolePosition.setToZero(soleZUpFrames.get(footstep.getRobotSide().getOppositeSide()));
         tempSolePosition.changeFrame(footstep.getFootstepPose().getReferenceFrame());
         footstep.setZ(tempSolePosition.getZ() + blindFootstepsHeightOffset.getValue());
         return true;
      }
      return false;
   }

   public void requestSwing(RobotSide upcomingSwingSide, Footstep footstep, double swingTime)
   {
      requestSwing(upcomingSwingSide, footstep, swingTime, null, null);
   }

   public void requestSwing(RobotSide upcomingSwingSide,
                            Footstep footstep,
                            double swingTime,
                            FrameVector3DReadOnly finalCoMVelocity,
                            FrameVector3DReadOnly finalCoMAcceleration)
   {
      FootControlModule footControlModule = footControlModules.get(upcomingSwingSide);
      footControlModule.setFootstep(footstep, swingTime, finalCoMVelocity, finalCoMAcceleration);
      setContactStateForSwing(upcomingSwingSide);
   }

   public void initializeSwingTrajectoryPreview(RobotSide upcomingSwingSide, Footstep footstep, double swingTime)
   {
      footControlModules.get(upcomingSwingSide).initializeSwingTrajectoryPreview(footstep, swingTime);
   }

   public void updateSwingTrajectoryPreview(RobotSide upcomingSwingSide)
   {
      footControlModules.get(upcomingSwingSide).updateSwingTrajectoryPreview();
   }

   public void saveCurrentPositionsAsLastFootstepPositions()
   {
      for (RobotSide robotSide : RobotSide.values)
         footControlModules.get(robotSide).saveCurrentPositionAsLastFootstepPosition();
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command)
   {
      RobotSide robotSide = command.getRobotSide();
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.handleFootTrajectoryCommand(command);

      if (footControlModule.getCurrentConstraintType() != ConstraintType.MOVE_VIA_WAYPOINTS)
         setContactStateForMoveViaWaypoints(robotSide);
   }

   public void handleLegTrajectoryCommand(LegTrajectoryCommand command)
   {
      RobotSide robotSide = command.getRobotSide();
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.handleLegTrajectoryCommand(command);

      if (footControlModule.getCurrentConstraintType() != ConstraintType.MOVE_VIA_WAYPOINTS)
         setContactStateForMoveViaWaypoints(robotSide);
   }

   public ConstraintType getCurrentConstraintType(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getCurrentConstraintType();
   }

   public AbstractFootControlState getCurrentControlState(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getCurrentControlState();
   }

   public void adjustSwingTrajectory(RobotSide swingSide, Footstep adjustedFootstep, double swingTime)
   {
      adjustSwingTrajectory(swingSide, adjustedFootstep, null, null, swingTime);
   }

   public void adjustSwingTrajectory(RobotSide swingSide,
                                     Footstep adjustedFootstep,
                                     FrameVector3DReadOnly finalCoMVelocity,
                                     FrameVector3DReadOnly finalCoMAcceleration,
                                     double swingTime)
   {
//      if (!getCurrentConstraintType(swingSide).isLoadBearing())
         footControlModules.get(swingSide).setAdjustedFootstepAndTime(adjustedFootstep, finalCoMVelocity, finalCoMAcceleration, swingTime);
   }

   public void requestMoveStraightTouchdownForDisturbanceRecovery(RobotSide swingSide)
   {
      footControlModules.get(swingSide).requestTouchdownForDisturbanceRecovery();
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if (!command.isStopAllTrajectory())
         return;

      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.requestStopTrajectoryIfPossible();
      }
   }

   public void correctCoMHeightForSupportSingularityAvoidance(double zCurrent, CoMHeightTimeDerivativesDataBasics comHeightData)
   {
      // Correct, if necessary, the CoM height trajectory to avoid straight knee
      boolean leftCorrectionApplied = false, rightCorrectionApplied = false;
      for (RobotSide robotSide : RobotSide.values)
      {
         CoMHeightTimeDerivativesDataBasics sidedCoMHeightData = legComHeightData.get(robotSide);
         sidedCoMHeightData.set(comHeightData);
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.updateLegSingularityModule();
         boolean correctionApplied = footControlModule.correctCoMHeightTrajectoryForSupportSingularityAvoidance(sidedCoMHeightData, zCurrent, pelvisZUpFrame);
         if (robotSide == RobotSide.LEFT)
            leftCorrectionApplied = correctionApplied;
         else
            rightCorrectionApplied = correctionApplied;
      }

      if (leftCorrectionApplied != rightCorrectionApplied)
      {
         if (leftCorrectionApplied)
            comHeightData.set(leftLegCoMHeightData);
         else
            comHeightData.set(rightLegCoMHeightData);
      }
      else
      {
         WorkspaceLimiterReconciler.reconcileWorkspaceLimitedData(legComHeightData, comHeightData);
      }
   }

   public void correctCoMHeightForUnreachableFootstep(CoMHeightTimeDerivativesDataBasics comHeightData)
   {
      // Do that after to make sure the swing foot will land
      boolean leftCorrectionApplied = false, rightCorrectionApplied = false;
      for (RobotSide robotSide : RobotSide.values)
      {
         CoMHeightTimeDerivativesDataBasics sidedCoMHeightData = legComHeightData.get(robotSide);
         sidedCoMHeightData.set(comHeightData);
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.updateLegSingularityModule();
         boolean correctionApplied = footControlModule.correctCoMHeightTrajectoryForUnreachableFootStep(sidedCoMHeightData);
         if (robotSide == RobotSide.LEFT)
            leftCorrectionApplied = correctionApplied;
         else
            rightCorrectionApplied = correctionApplied;
      }

      if (leftCorrectionApplied != rightCorrectionApplied)
      {
         if (leftCorrectionApplied)
            comHeightData.set(leftLegCoMHeightData);
         else
            comHeightData.set(rightLegCoMHeightData);
      }
      else
      {
         WorkspaceLimiterReconciler.reconcileWorkspaceLimitedData(legComHeightData, comHeightData);
      }
   }

   public void initializeContactStatesForDoubleSupport(RobotSide transferToSide)
   {
      if (transferToSide == null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            setFlatFootContactState(robotSide);
         }
      }
      else
      {
         if (getCurrentConstraintType(transferToSide.getOppositeSide()) == ConstraintType.SWING) // That case happens when doing 2 steps on same side
            setFlatFootContactState(transferToSide.getOppositeSide());
         setFlatFootContactState(transferToSide); // still need to determine contact state for trailing leg. This is done in doAction as soon as the previous ICP trajectory is done
      }

      reset();
   }

   public void updateContactStatesInDoubleSupport(RobotSide transferToSide)
   {
      if (transferToSide == null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            if (getCurrentConstraintType(robotSide) == ConstraintType.TOES)
               setFlatFootContactState(robotSide);
         }
      }
      else
      {
         if (getCurrentConstraintType(transferToSide) == ConstraintType.TOES)
            setFlatFootContactState(transferToSide);
      }
   }

   private final FrameVector3D footNormalContactVector = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);

   public void setOnToesContactState(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      if (footControlModule.isInFlatSupportState())
      {
         footNormalContactVector.setIncludingFrame(feet.get(robotSide).getSoleFrame(), 0.0, 0.0, 1.0);
         footNormalContactVector.changeFrame(worldFrame);
      }
      else
      {
         footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      }

      footControlModule.setContactState(ConstraintType.TOES, footNormalContactVector);
   }

   public void setFlatFootContactState(RobotSide robotSide)
   {
      if (useWorldSurfaceNormalWhenFullyConstrained.getValue())
         footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      else
         footNormalContactVector.setIncludingFrame(feet.get(robotSide).getSoleFrame(), 0.0, 0.0, 1.0);
      footControlModules.get(robotSide).setContactState(ConstraintType.FULL, footNormalContactVector);

      if (footControlModules.get(robotSide).getCurrentConstraintType() == ConstraintType.TOES)
         controllerToolbox.restorePreviousFootContactPoints(robotSide);
   }

   public void setContactStateForSwing(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setContactState(ConstraintType.SWING);
   }

   private void setContactStateForMoveViaWaypoints(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setContactState(ConstraintType.MOVE_VIA_WAYPOINTS);
   }

   public double getExtraCoMMaxHeightWithToes()
   {
      return extraCoMMaxHeightWithToes.getValue();
   }

   public boolean isSteppingUp()
   {
      return toeOffManager.isSteppingUp();
   }

   /**
    * Checks whether the next footstep in {@param nextFootstep} is in correct location to achieve toe
    * off. Calls
    * {@link GeometricToeOffManager#areFeetWellPositionedForToeOff(RobotSide, FramePose3DReadOnly)}.
    *
    * @param transferToSide upcoming support side.
    * @return whether or not the footstep location is ok.
    */
   public boolean canDoDoubleSupportToeOff(RobotSide transferToSide)
   {
      return toeOffManager.areFeetWellPositionedForToeOff(transferToSide.getOppositeSide());
   }

   /**
    * Checks whether the next footstep in {@param nextFootstep} is in correct location to achieve toe
    * off. Calls
    * {@link GeometricToeOffManager#areFeetWellPositionedForToeOff(RobotSide, FramePose3DReadOnly)}.
    *
    * @param nextFootstepPose footstep to consider.
    * @param swingSide        upcoming support side.
    * @return whether the footstep location is ok.
    */
   public boolean canDoSingleSupportToeOff(FramePose3DReadOnly nextFootstepPose, RobotSide swingSide)
   {
      return toeOffManager.areFeetWellPositionedForToeOff(swingSide.getOppositeSide(), nextFootstepPose);
   }

   /**
    * <p>
    * Checks whether or not the robot state is proper for toe-off when in double support, and sets the
    * {@link GeometricToeOffManager#doToeOff()} variable accordingly.
    * </p>
    * <p>
    * These checks include:
    * </p>
    * <ol>
    * <li>doToeOffIfPossibleInDoubleSupport</li>
    * <li>desiredICP location being within the leading foot base of support.</li>
    * <li>currentICP location being within the leading foot base of support.</li>
    * <li>needToSwitchToToeOffForAnkleLimit</li>
    * </ol>
    * <p>
    * If able and the ankles are at the joint limits, transitions to toe-off. Then checks the current
    * state being with the base of support. Then checks the positioning of the leading leg to determine
    * if it is acceptable.
    * </p>
    *
    * @param nextFootstep next desired footstep to take.
    * @param exitCMP      exit CMP in the current foot
    * @param desiredECMP  current desired ECMP from ICP feedback.
    * @param desiredICP   current desired ICP from the reference trajectory.
    * @param currentICP   current ICP based on the robot state.
    */
   public void updateToeOffStatusSingleSupport(Footstep nextFootstep,
                                               FramePoint3DReadOnly exitCMP,
                                               FramePoint2DReadOnly desiredECMP,
                                               FramePoint2DReadOnly desiredICP,
                                               FramePoint2DReadOnly currentICP)
   {
      toeOffManager.updateToeOffStatusSingleSupport(nextFootstep.getRobotSide(),
                                                    nextFootstep.getFootstepPose(),
                                                    nextFootstep.getPredictedContactPoints(),
                                                    exitCMP,
                                                    desiredECMP,
                                                    desiredICP,
                                                    currentICP);
   }

   /**
    * <p>
    * Checks whether or not the robot state is proper for toe-off when in double support, and sets the
    * {@link GeometricToeOffManager#doToeOff()} ()} variable accordingly.
    * </p>
    * <p>
    * These checks include:
    * </p>
    * <ol>
    * <li>doToeOffIfPossibleInDoubleSupport</li>
    * <li>desiredICP location being within the leading foot base of support.</li>
    * <li>currentICP location being within the leading foot base of support.</li>
    * <li>needToSwitchToToeOffForAnkleLimit</li>
    * </ol>
    * <p>
    * If able and the ankles are at the joint limits, transitions to toe-off. Then checks the current
    * state being with the base of support. Then checks the positioning of the leading leg to determine
    * if it is acceptable.
    * </p>
    *
    * @param trailingLeg robot side for the trailing leg
    * @param desiredECMP current desired ECMP from ICP feedback.
    * @param desiredICP  current desired ICP from the reference trajectory.
    * @param currentICP  current ICP based on the robot state.
    */
   public void updateToeOffStatusDoubleSupport(RobotSide trailingLeg,
                                               FramePoint3DReadOnly exitCMP,
                                               FramePoint2DReadOnly desiredECMP,
                                               FramePoint2DReadOnly desiredICP,
                                               FramePoint2DReadOnly currentICP)
   {
      toeOffManager.updateToeOffStatusDoubleSupport(trailingLeg, exitCMP, desiredECMP, desiredICP, currentICP);
   }

   /**
    * Returns whether the current robot state is ok toe-off using a point toe contact. The checks for
    * this are called in either
    * {@link #updateToeOffStatusDoubleSupport(RobotSide, FramePoint3DReadOnly, FramePoint2DReadOnly, FramePoint2DReadOnly, FramePoint2DReadOnly)}
    * or
    * {@link #updateToeOffStatusSingleSupport(Footstep, FramePoint3DReadOnly, FramePoint2DReadOnly, FramePoint2DReadOnly, FramePoint2DReadOnly)},
    * based on the walking state. Calls {@link GeometricToeOffManager#doToeOff()}}.
    */
   public boolean okForPointToeOff(boolean isInSingleSupport)
   {
      if (!toeOffManager.doToeOff())
         return false;

      if (isInSingleSupport)
         return !toeOffParameters.useToeOffLineContactInSwing();
      else
         return !toeOffParameters.useToeOffLineContactInTransfer();
   }

   /**
    * Returns whether the current robot state is ok toe-off using a line toe contact. The checks for
    * this are called in either
    * {@link #updateToeOffStatusDoubleSupport(RobotSide, FramePoint3DReadOnly, FramePoint2DReadOnly, FramePoint2DReadOnly, FramePoint2DReadOnly)}
    * or
    * {@link #updateToeOffStatusSingleSupport(Footstep, FramePoint3DReadOnly, FramePoint2DReadOnly, FramePoint2DReadOnly, FramePoint2DReadOnly)},
    * based on the walking state. Calls {@link GeometricToeOffManager#doToeOff()}}.
    */
   public boolean okForLineToeOff(boolean isInSingleSupport)
   {
      if (!toeOffManager.doToeOff())
         return false;

      if (isInSingleSupport)
         return toeOffParameters.useToeOffLineContactInSwing();
      else
         return toeOffParameters.useToeOffLineContactInTransfer();
   }

   public boolean useToeLineContactInTransfer()
   {
      return toeOffParameters.useToeOffLineContactInTransfer();
   }

   public boolean isUsingPointContactInToeOff(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).isUsingPointContactInToeOff();
   }

   /**
    * Computes the desired toe off point, and sets the contact state in the foot control module to the
    * toe off state.
    *
    * @param trailingLeg trailing leg in the state
    * @param exitCMP     exit CMP from the ICP plan in the stance foot
    * @param desiredCMP  current desired CMP location
    */
   public void requestPointToeOff(RobotSide trailingLeg, FramePoint3DReadOnly exitCMP, FramePoint2DReadOnly desiredCMP)
   {
      toeOffCalculator.setExitCMP(exitCMP, trailingLeg);
      toeOffCalculator.computeToeOffContactPoint(desiredCMP, trailingLeg);
      footControlModules.get(trailingLeg).setUsePointContactInToeOff(true);
      requestToeOff(trailingLeg);
      controllerToolbox.updateBipedSupportPolygons();
   }

   /**
    * Computes the desired toe off line, and sets the contact state in the foot control module to the
    * toe off state.
    *
    * @param trailingLeg trailing leg in the state
    * @param exitCMP     exit CMP from the ICP plan in the stance foot
    * @param desiredCMP  current desired CMP location
    */
   public void requestLineToeOff(RobotSide trailingLeg, FramePoint3DReadOnly exitCMP, FramePoint2DReadOnly desiredCMP)
   {
      toeOffCalculator.setExitCMP(exitCMP, trailingLeg);
      toeOffCalculator.computeToeOffContactLine(desiredCMP, trailingLeg);
      footControlModules.get(trailingLeg).setUsePointContactInToeOff(false);
      requestToeOff(trailingLeg);
      controllerToolbox.updateBipedSupportPolygons();
   }

   private void requestToeOff(RobotSide trailingLeg)
   {
      if (footControlModules.get(trailingLeg).isInToeOff())
         return;
      setOnToesContactState(trailingLeg);
   }

   public void reset()
   {
      toeOffManager.reset();
   }

   public void resetHeightCorrectionParametersForSingularityAvoidance()
   {
      for (RobotSide robotSide : RobotSide.values)
         footControlModules.get(robotSide).resetHeightCorrectionParametersForSingularityAvoidance();
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor. It is clamped w.r.t. to
    * {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * 
    * @param speedUpFactor multiplier on the current time
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(RobotSide robotSide, double speedUpFactor)
   {
      return footControlModules.get(robotSide).requestSwingSpeedUp(speedUpFactor);
   }

   /**
    * Computes and returns the swing fraction through the active swing, including the speed up factor.
    *
    * @return the estimated swing time remaining.
    */
   public double getFractionThroughSwing(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getFractionThroughSwing();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getInverseDynamicsCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getFeedbackControlCommand();
   }

   public JointDesiredOutputListReadOnly getJointDesiredData(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getJointDesiredData();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (RobotSide robotSide : RobotSide.values)
      {
         FeedbackControlCommandList template = footControlModules.get(robotSide).createFeedbackControlTemplate();
         for (int i = 0; i < template.getNumberOfCommands(); i++)
            ret.addCommand(template.getCommand(i));
      }
      return ret;
   }

   public void initializeFootExploration(RobotSide robotSideToExplore)
   {
      if (robotSideToExplore == null)
         return;
      FootControlModule footControlModule = footControlModules.get(robotSideToExplore);
      footControlModule.initializeFootExploration();
   }

   public boolean isFootToeingOffSlipping(RobotSide robotSideToCheck)
   {
      return footControlModules.get(robotSideToCheck).isFootToeingOffSlipping();
   }

   public void unload(RobotSide sideToUnload, double percentInUnloading, double rhoMin)
   {
      footControlModules.get(sideToUnload).unload(percentInUnloading, rhoMin);
   }

   public void resetLoadConstraints(RobotSide sideToUnload)
   {
      footControlModules.get(sideToUnload).resetLoadConstraints();
   }

   public Object pollStatusToReport(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).pollStatusToReport();
   }

   public void liftOff(RobotSide side, double pitch, double pitchVelocity, double duration)
   {
      footControlModules.get(side).liftOff(pitch, pitchVelocity, duration);
   }

   public void touchDown(RobotSide side, double initialPitch, double initialPitchVelocity, double pitch, double duration)
   {
      setFlatFootContactState(side);
      footControlModules.get(side).touchDown(initialPitch, initialPitchVelocity, pitch, duration);
   }

   public MultipleWaypointsPoseTrajectoryGenerator getSwingTrajectory(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getSwingTrajectory();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(toeOffCalculator.getSCS2YoGraphics());
      for (RobotSide robotSide : RobotSide.values)
         group.addChild(footControlModules.get(robotSide).getSCS2YoGraphics());
      return group;
   }
}
