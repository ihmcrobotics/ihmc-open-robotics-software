package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnTheEdgesManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class FeetManager
{
   private static final boolean USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<FootControlModule> footControlModules = new SideDependentList<>();

   private final ToeOffHelper toeOffHelper;
   private final WalkOnTheEdgesManager walkOnTheEdgesManager;

   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final ReferenceFrame pelvisZUpFrame;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final HighLevelHumanoidControllerToolbox momentumBasedController;

   private final BooleanYoVariable attemptToStraightenLegs = new BooleanYoVariable("attemptToStraightenLegs", registry);

   // TODO Needs to be cleaned up someday... (Sylvain)
   public FeetManager(HighLevelHumanoidControllerToolbox momentumBasedController, WalkingControllerParameters walkingControllerParameters,
         YoVariableRegistry parentRegistry)
   {
      this.momentumBasedController = momentumBasedController;
      feet = momentumBasedController.getContactableFeet();

      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
         contactStates.put(robotSide, momentumBasedController.getContactState(feet.get(robotSide)));

      toeOffHelper = new ToeOffHelper(contactStates, feet, walkingControllerParameters, registry);
      walkOnTheEdgesManager = new WalkOnTheEdgesManager(momentumBasedController, toeOffHelper, walkingControllerParameters, feet, registry);

      this.footSwitches = momentumBasedController.getFootSwitches();
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();

      YoSE3PIDGainsInterface swingFootControlGains = walkingControllerParameters.createSwingFootControlGains(registry);
      YoSE3PIDGainsInterface holdPositionFootControlGains = walkingControllerParameters.createHoldPositionFootControlGains(registry);
      YoSE3PIDGainsInterface toeOffFootControlGains = walkingControllerParameters.createToeOffFootControlGains(registry);
      YoSE3PIDGainsInterface edgeTouchdownFootControlGains = walkingControllerParameters.createEdgeTouchdownFootControlGains(registry);

      walkingControllerParameters.getOrCreateExplorationParameters(registry);
      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = new FootControlModule(robotSide, toeOffHelper, walkingControllerParameters, swingFootControlGains,
               holdPositionFootControlGains, toeOffFootControlGains, edgeTouchdownFootControlGains, momentumBasedController, registry);
         footControlModule.setAttemptToStraightenLegs(walkingControllerParameters.attemptToStraightenLegs());

         footControlModules.put(robotSide, footControlModule);
      }

      attemptToStraightenLegs.set(walkingControllerParameters.attemptToStraightenLegs());
      attemptToStraightenLegs.addVariableChangedListener(new VariableChangedListener()
      {
         @Override public void variableChanged(YoVariable<?> v)
         {
            for (RobotSide robotSide : RobotSide.values)
               footControlModules.get(robotSide).setAttemptToStraightenLegs(attemptToStraightenLegs.getBooleanValue());
         }
      });

      parentRegistry.addChild(registry);
   }

   public void setWeights(double highFootWeight, double defaultFootWeight)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.setWeights(highFootWeight, defaultFootWeight);
      }
   }

   public void setWeights(Vector3D highAngularFootWeight, Vector3D highLinearFootWeight, Vector3D defaultAngularFootWeight, Vector3D defaultLinearFootWeight)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.setWeights(highAngularFootWeight, highLinearFootWeight, defaultAngularFootWeight, defaultLinearFootWeight);
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

   public void requestSwing(RobotSide upcomingSwingSide, Footstep footstep, double swingTime)
   {
      if (!footstep.getTrustHeight())
      {
         FramePoint supportAnklePosition = new FramePoint(ankleZUpFrames.get(upcomingSwingSide.getOppositeSide()));
         supportAnklePosition.changeFrame(footstep.getParentFrame());
         double newHeight = supportAnklePosition.getZ();
         footstep.setZ(newHeight);
      }

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

   public void replanSwingTrajectory(RobotSide swingSide, Footstep footstep, double swingTime, boolean continuousReplan)
   {
      footControlModules.get(swingSide).replanTrajectory(footstep, swingTime, continuousReplan);
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

   public boolean isInFlatSupportState(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).isInFlatSupportState();
   }

   public void correctCoMHeight(FrameVector2d desiredICPVelocity, double zCurrent, CoMHeightTimeDerivativesData comHeightData)
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

   private final FrameVector footNormalContactVector = new FrameVector(worldFrame, 0.0, 0.0, 1.0);

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
         momentumBasedController.restorePreviousFootContactPoints(robotSide);

      FootControlModule supportFootControlModule = footControlModules.get(robotSide.getOppositeSide());
      supportFootControlModule.setAllowFootholdAdjustments(true);
   }

   private void setContactStateForSwing(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setContactState(ConstraintType.SWING);

      FootControlModule supportFootControlModule = footControlModules.get(robotSide.getOppositeSide());
      supportFootControlModule.setAllowFootholdAdjustments(false);
   }

   private void setContactStateForMoveViaWaypoints(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setContactState(ConstraintType.MOVE_VIA_WAYPOINTS);
   }

   public WalkOnTheEdgesManager getWalkOnTheEdgesManager()
   {
      return walkOnTheEdgesManager;
   }

   public boolean willDoToeOff(Footstep nextFootstep, RobotSide transferToSide)
   {
      return walkOnTheEdgesManager.willDoToeOff(nextFootstep, transferToSide);
   }

   /**
    * {@link WalkOnTheEdgesManager#updateToeOffStatus(RobotSide, FramePoint, FramePoint2d, FramePoint2d, FramePoint2d)}
    * @return {@link WalkOnTheEdgesManager#doToeOff}
    */
   public boolean checkIfToeOffSafe(RobotSide trailingLeg, FramePoint exitCMP, FramePoint2d desiredECMP, FramePoint2d desiredICP, FramePoint2d currentICP)
   {
      walkOnTheEdgesManager.inDoubleSupport();
      walkOnTheEdgesManager.updateToeOffStatus(trailingLeg, exitCMP, desiredECMP, desiredICP, currentICP);

      return walkOnTheEdgesManager.doToeOff();
   }

   /**
    * {@link WalkOnTheEdgesManager#updateToeOffStatus(RobotSide, FramePoint, FramePoint2d, FramePoint2d, FramePoint2d)}.
    * @return {@link WalkOnTheEdgesManager#doToeOff}
    */
   public boolean checkIfToeOffSafeSingleSupport(Footstep nextFootstep, FramePoint exitCMP, FramePoint2d desiredECMP, FramePoint2d currentICP, FramePoint2d desiredICP)
   {
      RobotSide trailingLeg = nextFootstep.getRobotSide().getOppositeSide();
      walkOnTheEdgesManager.inSingleSupport();
      walkOnTheEdgesManager.submitNextFootstep(nextFootstep);
      walkOnTheEdgesManager.updateToeOffStatus(trailingLeg, exitCMP, desiredECMP, desiredICP, currentICP);

      return walkOnTheEdgesManager.doToeOff();
   }

   public void requestToeOff(RobotSide trailingLeg)
   {
      if (footControlModules.get(trailingLeg).isInToeOff())
         return;
      setOnToesContactState(trailingLeg);
   }

   public void computeToeOffContactPoint(RobotSide trailingLeg, FramePoint exitCMP, FramePoint2d desiredCMP)
   {
      toeOffHelper.setExitCMP(exitCMP, trailingLeg);
      toeOffHelper.computeToeOffContactPoint(desiredCMP, trailingLeg);
   }

   public void reset()
   {
      walkOnTheEdgesManager.reset();
   }

   public boolean doToeOffIfPossible()
   {
      return walkOnTheEdgesManager.doToeOffIfPossible();
   }

   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return walkOnTheEdgesManager.doToeOffIfPossibleInSingleSupport();
   }

   public void resetHeightCorrectionParametersForSingularityAvoidance()
   {
      for (RobotSide robotSide : RobotSide.values)
         footControlModules.get(robotSide).resetHeightCorrectionParametersForSingularityAvoidance();
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @param speedUpFactor
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
}
