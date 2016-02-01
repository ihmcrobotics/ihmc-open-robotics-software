package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnTheEdgesManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.YoSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FeetManager
{
   private static final boolean USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<FootControlModule> footControlModules = new SideDependentList<>();

   private final WalkOnTheEdgesManager walkOnTheEdgesManager;

   private final SideDependentList<? extends ContactablePlaneBody> feet;

   private final ReferenceFrame pelvisZUpFrame;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSwingLeg = new DoubleYoVariable("singularityEscapeNullspaceMultiplierSwingLeg", registry);
   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSupportLeg = new DoubleYoVariable("singularityEscapeNullspaceMultiplierSupportLeg", registry);
   private final DoubleYoVariable singularityEscapeNullspaceMultiplierSupportLegLocking = new DoubleYoVariable("singularityEscapeNullspaceMultiplierSupportLegLocking", registry);

   // TODO Needs to be cleaned up someday... (Sylvain)
   public FeetManager(MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters,
         DoubleProvider swingTimeProvider, YoVariableRegistry parentRegistry)
   {
      double singularityEscapeMultiplierForSwing = walkingControllerParameters.getSwingSingularityEscapeMultiplier();
      singularityEscapeNullspaceMultiplierSwingLeg.set(singularityEscapeMultiplierForSwing);
      singularityEscapeNullspaceMultiplierSupportLeg.set(walkingControllerParameters.getSupportSingularityEscapeMultiplier());
      singularityEscapeNullspaceMultiplierSupportLegLocking.set(0.0); // -0.5);

      feet = momentumBasedController.getContactableFeet();
      walkOnTheEdgesManager = new WalkOnTheEdgesManager(momentumBasedController, walkingControllerParameters, feet, footControlModules, registry);

      this.footSwitches = momentumBasedController.getFootSwitches();
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();

      YoSE3PIDGains swingFootControlGains = walkingControllerParameters.createSwingFootControlGains(registry);
      YoSE3PIDGains holdPositionFootControlGains = walkingControllerParameters.createHoldPositionFootControlGains(registry);
      YoSE3PIDGains toeOffFootControlGains = walkingControllerParameters.createToeOffFootControlGains(registry);
      YoSE3PIDGains edgeTouchdownFootControlGains = walkingControllerParameters.createEdgeTouchdownFootControlGains(registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         FootControlModule footControlModule = new FootControlModule(robotSide, walkingControllerParameters, swingFootControlGains,
               holdPositionFootControlGains, toeOffFootControlGains, edgeTouchdownFootControlGains, swingTimeProvider, momentumBasedController, registry);
         footControlModule.setNullspaceMultiplier(singularityEscapeMultiplierForSwing);

         footControlModules.put(robotSide, footControlModule);
      }

      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footSwitches.get(robotSide).hasFootHitGround(); //debug
         footControlModules.get(robotSide).doControl();
      }
   }

   public void requestSwing(RobotSide upcomingSwingSide, Footstep footstep)
   {
      if (!footstep.getTrustHeight())
      {
         FramePoint supportAnklePosition = new FramePoint(ankleZUpFrames.get(upcomingSwingSide.getOppositeSide()));
         supportAnklePosition.changeFrame(footstep.getParentFrame());
         double newHeight = supportAnklePosition.getZ();
         footstep.setZ(newHeight);
      }

      walkOnTheEdgesManager.updateEdgeTouchdownStatus(upcomingSwingSide.getOppositeSide(), footstep);
      FootControlModule footControlModule = footControlModules.get(upcomingSwingSide);

      if (walkOnTheEdgesManager.willLandOnHeel())
      {
         walkOnTheEdgesManager.modifyFootstepForEdgeTouchdown(footstep, footControlModule.getHeelTouchdownInitialAngle());
      }
      else if (walkOnTheEdgesManager.willLandOnToes())
      {
         walkOnTheEdgesManager.modifyFootstepForEdgeTouchdown(footstep, footControlModule.getToeTouchdownInitialAngle());
      }

      footControlModule.setFootstep(footstep);
      setContactStateForSwing(upcomingSwingSide);
   }

   public void requestMoveStraight(RobotSide robotSide, FramePose footPose, double trajectoryTime)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setFootPose(footPose, trajectoryTime);
      if (footControlModule.getCurrentConstraintType() == ConstraintType.MOVE_STRAIGHT)
         footControlModule.resetCurrentState();
      else
         setContactStateForMoveStraight(robotSide);
   }

   public boolean isInEdgeTouchdownState(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).isInEdgeTouchdownState();
   }

   public ConstraintType getCurrentConstraintType(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getCurrentConstraintType();
   }

   public void replanSwingTrajectory(RobotSide swingSide, Footstep footstep)
   {
      footControlModules.get(swingSide).replanTrajectory(footstep);
   }

   public void requestMoveStraightTouchdownForDisturbanceRecovery(RobotSide swingSide)
   {
      footControlModules.get(swingSide).requestMoveStraightTouchdownForDisturbanceRecovery();
   }

   public boolean isInSingularityNeighborhood(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).isInSingularityNeighborhood();
   }

   public void doSingularityEscape(RobotSide robotSide)
   {
      footControlModules.get(robotSide).doSingularityEscape(true);
   }

   public void doSingularityEscape(RobotSide robotSide, double temporarySingularityEscapeNullspaceMultiplier)
   {
      footControlModules.get(robotSide).doSingularityEscape(temporarySingularityEscapeNullspaceMultiplier);
   }

   public boolean isInFlatSupportState(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).isInFlatSupportState();
   }

   public void correctCoMHeight(RobotSide trailingLeg, FrameVector2d desiredICPVelocity, double zCurrent, CoMHeightTimeDerivativesData comHeightData)
   {
      RobotSide[] leadingLegFirst;
      if (trailingLeg != null)
         leadingLegFirst = new RobotSide[] { trailingLeg.getOppositeSide(), trailingLeg };
      else
         leadingLegFirst = RobotSide.values;

      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).updateLegSingularityModule();
      }

      // Correct, if necessary, the CoM height trajectory to avoid the knee to collapse
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).correctCoMHeightTrajectoryForCollapseAvoidance(desiredICPVelocity, comHeightData, zCurrent, pelvisZUpFrame,
               footSwitches.get(robotSide).computeFootLoadPercentage());
      }

      // Correct, if necessary, the CoM height trajectory to avoid straight knee
      for (RobotSide robotSide : leadingLegFirst)
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
            if (!isInEdgeTouchdownState(robotSide))
               setFlatFootContactState(robotSide);
         }
      }
      else if (willLandOnToes())
      {
         setTouchdownOnToesContactState(transferToSide);
      }
      else if (willLandOnHeel())
      {
         setTouchdownOnHeelContactState(transferToSide);
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
            if ((isInEdgeTouchdownState(robotSide) && isEdgeTouchDownDone(robotSide)) || (getCurrentConstraintType(robotSide) == ConstraintType.TOES))
               setFlatFootContactState(robotSide);
         }
      }
      else
      {
         if ((isInEdgeTouchdownState(transferToSide) && isEdgeTouchDownDone(transferToSide))
               || (getCurrentConstraintType(transferToSide) == ConstraintType.TOES))
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

   private void setTouchdownOnHeelContactState(RobotSide robotSide)
   {
      footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      footControlModules.get(robotSide).setContactState(ConstraintType.HEEL_TOUCHDOWN, footNormalContactVector);
   }

   public void setTouchdownOnToesContactState(RobotSide robotSide)
   {
      footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      footControlModules.get(robotSide).setContactState(ConstraintType.TOES_TOUCHDOWN, footNormalContactVector);
   }

   public void setFlatFootContactState(RobotSide robotSide)
   {
      if (USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED)
         footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      else
         footNormalContactVector.setIncludingFrame(feet.get(robotSide).getSoleFrame(), 0.0, 0.0, 1.0);
      footControlModules.get(robotSide).setContactState(ConstraintType.FULL, footNormalContactVector);
   }

   private void setContactStateForSwing(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setContactState(ConstraintType.SWING);
   }

   private void setContactStateForMoveStraight(RobotSide robotSide)
   {
      FootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.doSingularityEscapeBeforeTransitionToNextState();
      footControlModule.setContactState(ConstraintType.MOVE_STRAIGHT);
   }

   public WalkOnTheEdgesManager getWalkOnTheEdgesManager()
   {
      return walkOnTheEdgesManager;
   }

   public boolean isEdgeTouchDownDone(RobotSide robotSide)
   {
      return walkOnTheEdgesManager.isEdgeTouchDownDone(robotSide);
   }

   public boolean willDoToeOff(Footstep nextFootstep, RobotSide transferToSide)
   {
      return walkOnTheEdgesManager.willDoToeOff(nextFootstep, transferToSide);
   }

   public boolean checkIfToeOffSafe(RobotSide trailingLeg, FramePoint2d desiredECMP, FramePoint2d desiredICP, FramePoint2d currentICP)
   {
      walkOnTheEdgesManager.updateToeOffStatus(trailingLeg, desiredECMP, desiredICP, currentICP);

      return walkOnTheEdgesManager.doToeOff();
   }

   public void requestToeOff(RobotSide trailingLeg, double predictedToeOffDuration)
   {
      if (footControlModules.get(trailingLeg).isInToeOff())
         return;
      footControlModules.get(trailingLeg).setPredictedToeOffDuration(predictedToeOffDuration);
      setOnToesContactState(trailingLeg);
   }

   public void registerDesiredContactPointForToeOff(RobotSide robotSide, FramePoint2d desiredContactPoint)
   {
      footControlModules.get(robotSide).registerDesiredContactPointForToeOff(desiredContactPoint);
   }

   public boolean willLandOnToes()
   {
      return walkOnTheEdgesManager.willLandOnToes();
   }

   public boolean willLandOnHeel()
   {
      return walkOnTheEdgesManager.willLandOnHeel();
   }

   public void reset()
   {
      walkOnTheEdgesManager.reset();
   }

   public void updateEdgeTouchdownStatus(RobotSide supportLeg, Footstep nextFootstep)
   {
      walkOnTheEdgesManager.updateEdgeTouchdownStatus(supportLeg, nextFootstep);
   }

   public double getHeelTouchdownInitialAngle(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getHeelTouchdownInitialAngle();
   }

   public double getToeTouchdownInitialAngle(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getToeTouchdownInitialAngle();
   }

   public boolean doToeOffIfPossible()
   {
      return walkOnTheEdgesManager.doToeOffIfPossible();
   }

   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return walkOnTheEdgesManager.doToeOffIfPossibleInSingleSupport();
   }

   public void lockKnee(RobotSide robotSide)
   {
      footControlModules.get(robotSide).doSingularityEscape(singularityEscapeNullspaceMultiplierSupportLegLocking.getDoubleValue());
   }

   public void doSupportSingularityEscape(RobotSide robotSide)
   {
      footControlModules.get(robotSide).doSingularityEscape(singularityEscapeNullspaceMultiplierSupportLeg.getDoubleValue());
   }

   public void resetCurrentState(RobotSide robotSide)
   {
      footControlModules.get(robotSide).resetCurrentState();
   }

   public void enableAnkleLimitAvoidanceInSupportState(RobotSide robotSide, boolean enable)
   {
      footControlModules.get(robotSide).enableAnkleLimitAvoidanceInSupportState(enable);
   }

   public void enableAnkleLimitAvoidanceInSupportState(boolean enable)
   {
      for (RobotSide robotSide : RobotSide.values)
         footControlModules.get(robotSide).enableAnkleLimitAvoidanceInSupportState(enable);
   }

   public boolean isLegDoingToeOffAndAtLimit(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).isLegDoingToeOffAndAtLimit();
   }

   public void resetHeightCorrectionParametersForSingularityAvoidance()
   {
      for (RobotSide robotSide : RobotSide.values)
         footControlModules.get(robotSide).resetHeightCorrectionParametersForSingularityAvoidance();
   }

   public void requestSwingSpeedUp(RobotSide robotSide, double speedUpFactor)
   {
      footControlModules.get(robotSide).requestSwingSpeedUp(speedUpFactor);
   }
}
