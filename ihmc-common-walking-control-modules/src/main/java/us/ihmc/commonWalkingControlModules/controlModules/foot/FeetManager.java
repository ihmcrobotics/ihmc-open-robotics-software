package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.CentroidProjectionToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ICPPlanToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.SimpleToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ToeOffEnum;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.WrapperForMultipleToeOffCalculators;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FeetManager
{
   private static final boolean USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<FootControlModule> footControlModules = new SideDependentList<>();

   private final ToeOffCalculator toeOffCalculator;
   private final ToeOffManager toeOffManager;

   private final SideDependentList<ContactableFoot> feet;

   private final ReferenceFrame pelvisZUpFrame;
   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final FramePoint3D tempSolePosition = new FramePoint3D();
   private final DoubleParameter blindFootstepsHeightOffset;

   public FeetManager(HighLevelHumanoidControllerToolbox controllerToolbox, WalkingControllerParameters walkingControllerParameters,
                      PIDSE3GainsReadOnly swingFootGains, PIDSE3GainsReadOnly holdFootGains, PIDSE3GainsReadOnly toeOffFootGains,
                      YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      feet = controllerToolbox.getContactableFeet();

      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
         contactStates.put(robotSide, controllerToolbox.getFootContactState(robotSide));

      ToeOffCalculator centroidProjectionCalculator = new CentroidProjectionToeOffCalculator(contactStates, feet, walkingControllerParameters.getToeOffParameters(), registry);
      ToeOffCalculator icpPlanCalculator = new ICPPlanToeOffCalculator(contactStates, feet, registry);
      ToeOffCalculator simpleCalculator = new SimpleToeOffCalculator(feet, registry);

      EnumMap<ToeOffEnum, ToeOffCalculator> toeOffCalculators = new EnumMap<>(ToeOffEnum.class);
      toeOffCalculators.put(centroidProjectionCalculator.getEnum(), centroidProjectionCalculator);
      toeOffCalculators.put(icpPlanCalculator.getEnum(), icpPlanCalculator);
      toeOffCalculators.put(simpleCalculator.getEnum(), simpleCalculator);

      toeOffCalculator = new WrapperForMultipleToeOffCalculators(toeOffCalculators, registry);

      toeOffManager = new ToeOffManager(controllerToolbox, toeOffCalculator, walkingControllerParameters, feet, registry);

      this.footSwitches = controllerToolbox.getFootSwitches();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      soleZUpFrames = referenceFrames.getSoleZUpFrames();

      ExplorationParameters explorationParameters = null;
      if (walkingControllerParameters.createFootholdExplorationTools())
      {
         explorationParameters = new ExplorationParameters(registry);
      }

      boolean enableSmoothUnloading = walkingControllerParameters.enforceSmoothFootUnloading();
      DoubleProvider minWeightFractionPerFoot = enableSmoothUnloading ? new DoubleParameter("minWeightFractionPerFoot", registry, 0.0) : null;
      DoubleProvider maxWeightFractionPerFoot = enableSmoothUnloading ? new DoubleParameter("maxWeightFractionPerFoot", registry, 2.0) : null;

      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = new FootControlModule(robotSide, toeOffCalculator, walkingControllerParameters, swingFootGains, holdFootGains,
                                                                     toeOffFootGains, controllerToolbox, explorationParameters, minWeightFractionPerFoot,
                                                                     maxWeightFractionPerFoot, registry);

         footControlModules.put(robotSide, footControlModule);
      }

      double defaultBlindFootstepsHeightOffset = walkingControllerParameters.getSwingTrajectoryParameters().getBlindFootstepsHeightOffset();
      blindFootstepsHeightOffset = new DoubleParameter("blindFootstepsHeightOffset", registry, defaultBlindFootstepsHeightOffset);

      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3DReadOnly loadedFootAngularWeight, Vector3DReadOnly loadedFootLinearWeight, Vector3DReadOnly footAngularWeight,
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
         footSwitches.get(robotSide).hasFootHitGround(); //debug
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
      FootControlModule footControlModule = footControlModules.get(upcomingSwingSide);
      footControlModule.setFootstep(footstep, swingTime);
      setContactStateForSwing(upcomingSwingSide);
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command)
   {
      RobotSide robotSide = command.getRobotSide();
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.handleFootTrajectoryCommand(command);

      if (footControlModule.getCurrentConstraintType() != ConstraintType.MOVE_VIA_WAYPOINTS)
         setContactStateForMoveViaWaypoints(robotSide);
   }

   public ConstraintType getCurrentConstraintType(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getCurrentConstraintType();
   }

   public void adjustSwingTrajectory(RobotSide swingSide, Footstep adjustedFootstep, double swingTime)
   {
      footControlModules.get(swingSide).setAdjustedFootstepAndTime(adjustedFootstep, swingTime);
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

   public void correctCoMHeight(FrameVector2D desiredICPVelocity, double zCurrent, CoMHeightTimeDerivativesData comHeightData)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).updateLegSingularityModule();
      }

      // Correct, if necessary, the CoM height trajectory to avoid straight knee
      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.correctCoMHeightTrajectoryForSingularityAvoidance(desiredICPVelocity, comHeightData, zCurrent, pelvisZUpFrame);
      }

      // Do that after to make sure the swing foot will land
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).correctCoMHeightTrajectoryForUnreachableFootStep(comHeightData);
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
      if (USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED)
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

   public ToeOffManager getToeOffManager()
   {
      return toeOffManager;
   }

   /**
    * Checks whether or not the next footstep in {@param nextFootstep} is in correct location to achieve toe off.
    * Calls {@link ToeOffManager#canDoDoubleSupportToeOff(Footstep, RobotSide)}.
    *
    * @param nextFootstep footstep to consider.
    * @param transferToSide upcoming support side.
    * @return whether or not the footstep location is ok.
    */
   public boolean canDoDoubleSupportToeOff(Footstep nextFootstep, RobotSide transferToSide)
   {
      return toeOffManager.canDoDoubleSupportToeOff(nextFootstep, transferToSide);
   }

   /**
    * Checks whether or not the next footstep in {@param nextFootstep} is in correct location to achieve toe off.
    * Calls {@link ToeOffManager#canDoSingleSupportToeOff(Footstep, RobotSide)}.
    *
    * @param nextFootstep footstep to consider.
    * @param transferToSide upcoming support side.
    * @return whether or not the footstep location is ok.
    */
   public boolean canDoSingleSupportToeOff(Footstep nextFootstep, RobotSide transferToSide)
   {
      return toeOffManager.canDoSingleSupportToeOff(nextFootstep, transferToSide);
   }

   /**
    * <p>
    * Checks whether or not the robot state is proper for toe-off when in double support, and sets the {@link ToeOffManager#doLineToeOff} variable accordingly.
    * </p>
    * <p>
    * These checks include:
    * </p>
    * <ol>
    *   <li>doToeOffIfPossibleInDoubleSupport</li>
    *   <li>desiredECMP location being within the support polygon account for toe-off, if {@link ToeOffParameters#checkECMPLocationToTriggerToeOff()} is true.</li>
    *   <li>desiredICP location being within the leading foot base of support.</li>
    *   <li>currentICP location being within the leading foot base of support.</li>
    *   <li>needToSwitchToToeOffForAnkleLimit</li>
    * </ol>
    * <p>
    * If able and the ankles are at the joint limits, transitions to toe-off. Then checks the current state being with the base of support. Then checks the
    * positioning of the leading leg to determine if it is acceptable.
    * </p>
    *
    * @param nextFootstep next desired footstep to take.
    * @param exitCMP exit CMP in the current foot
    * @param desiredECMP current desired ECMP from ICP feedback.
    * @param desiredICP current desired ICP from the reference trajectory.
    * @param currentICP current ICP based on the robot state.
    */
   public void updateToeOffStatusSingleSupport(Footstep nextFootstep, FramePoint3D exitCMP, FramePoint2D desiredECMP, FramePoint2D desiredCoP,
         FramePoint2D desiredICP, FramePoint2D currentICP)
   {
      toeOffManager.submitNextFootstep(nextFootstep);
      toeOffManager.updateToeOffStatusSingleSupport(exitCMP, desiredECMP, desiredCoP, desiredICP, currentICP);
   }

   /**
    * <p>
    * Checks whether or not the robot state is proper for toe-off when in double support, and sets the {@link ToeOffManager#doLineToeOff} variable accordingly.
    * </p>
    * <p>
    * These checks include:
    * </p>
    * <ol>
    *   <li>doToeOffIfPossibleInDoubleSupport</li>
    *   <li>desiredECMP location being within the support polygon account for toe-off, if {@link ToeOffParameters#checkECMPLocationToTriggerToeOff()} is true.</li>
    *   <li>desiredICP location being within the leading foot base of support.</li>
    *   <li>currentICP location being within the leading foot base of support.</li>
    *   <li>needToSwitchToToeOffForAnkleLimit</li>
    * </ol>
    * <p>
    * If able and the ankles are at the joint limits, transitions to toe-off. Then checks the current state being with the base of support. Then checks the
    * positioning of the leading leg to determine if it is acceptable.
    * </p>
    *
    * @param trailingLeg robot side for the trailing leg
    * @param desiredECMP current desired ECMP from ICP feedback.
    * @param desiredICP current desired ICP from the reference trajectory.
    * @param currentICP current ICP based on the robot state.
    */
   public void updateToeOffStatusDoubleSupport(RobotSide trailingLeg, Footstep nextFootstep, FramePoint3D exitCMP, FramePoint2D desiredECMP, FramePoint2D desiredCoP,
         FramePoint2D desiredICP, FramePoint2D currentICP)
   {
      toeOffManager.submitNextFootstep(nextFootstep);
      toeOffManager.updateToeOffStatusDoubleSupport(trailingLeg, exitCMP, desiredECMP, desiredCoP, desiredICP, currentICP);
   }

   /**
    * Returns whether or not the current robot state is ok toe-off using a point toe contact.
    * The checks for this are called in either {@link #updateToeOffStatusDoubleSupport(RobotSide, FramePoint3D, FramePoint2D, FramePoint2D, FramePoint2D, FramePoint2D)} or
    * {@link #updateToeOffStatusSingleSupport(Footstep, FramePoint3D, FramePoint2D, FramePoint2D, FramePoint2D, FramePoint2D)}, based on the walking state.
    * Calls {@link ToeOffManager#doPointToeOff}.
    */
   public boolean okForPointToeOff()
   {
      return toeOffManager.doPointToeOff();
   }

   /**
    * Returns whether or not the current robot state is ok toe-off using a line toe contact.
    * The checks for this are called in either {@link #updateToeOffStatusDoubleSupport(RobotSide, FramePoint3D, FramePoint2D, FramePoint2D, FramePoint2D, FramePoint2D)} or
    * {@link #updateToeOffStatusSingleSupport(Footstep, FramePoint3D, FramePoint2D, FramePoint2D, FramePoint2D, FramePoint2D)}, based on the walking state.
    * Calls {@link ToeOffManager#doLineToeOff}.
    */
   public boolean okForLineToeOff()
   {
      return toeOffManager.doLineToeOff();
   }

   /**
    * Computes the desired toe off point, and sets the contact state in the foot control module to the toe off state.
    *
    * @param trailingLeg trailing leg in the state
    * @param exitCMP exit CMP from the ICP plan in the stance foot
    * @param desiredCMP current desired CMP location
    */
   public void requestPointToeOff(RobotSide trailingLeg, FramePoint3D exitCMP, FramePoint2D desiredCMP)
   {
      toeOffCalculator.setExitCMP(exitCMP, trailingLeg);
      toeOffCalculator.computeToeOffContactPoint(desiredCMP, trailingLeg);
      footControlModules.get(trailingLeg).setUsePointContactInToeOff(true);
      requestToeOff(trailingLeg);
      controllerToolbox.updateBipedSupportPolygons();
   }

   /**
    * Computes the desired toe off line, and sets the contact state in the foot control module to the toe off state.
    *
    * @param trailingLeg trailing leg in the state
    * @param exitCMP exit CMP from the ICP plan in the stance foot
    * @param desiredCMP current desired CMP location
    */
   public void requestLineToeOff(RobotSide trailingLeg, FramePoint3D exitCMP, FramePoint2D desiredCMP)
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

   public boolean shouldComputeToeLineContact()
   {
      return toeOffManager.shouldComputeToeLineContact();
   }

   public boolean shouldComputeToePointContact()
   {
      return toeOffManager.shouldComputeToePointContact();
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
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @param speedUpFactor multiplier on the current time
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(RobotSide robotSide, double speedUpFactor)
   {
      return footControlModules.get(robotSide).requestSwingSpeedUp(speedUpFactor);
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
      if (robotSideToExplore == null) return;
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

   public void liftOff(RobotSide side, double pitch, double duration)
   {
      footControlModules.get(side).liftOff(pitch, duration);
   }

   public void touchDown(RobotSide side, double pitch, double duration)
   {
      footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      footControlModules.get(side).setContactState(ConstraintType.FULL, footNormalContactVector);
      footControlModules.get(side).touchDown(pitch, duration);
      reset();
   }
}
