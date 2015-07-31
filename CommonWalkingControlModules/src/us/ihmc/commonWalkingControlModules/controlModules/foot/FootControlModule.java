package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.robotics.humanoidRobot.partNames.LegJointName;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVelocityProvider;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateTransition;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

public class FootControlModule
{
   private static final boolean USE_HEURISTIC_SWING_STATE = false;

   private final YoVariableRegistry registry;
   private final ContactablePlaneBody contactableFoot;

   public enum ConstraintType
   {
      FULL, HOLD_POSITION, HEEL_TOUCHDOWN, TOES_TOUCHDOWN, TOES, SWING, MOVE_STRAIGHT
   }

   private static final double coefficientOfFriction = 0.8;

   private final StateMachine<ConstraintType> stateMachine;
   private final EnumMap<ConstraintType, boolean[]> contactStatesMap = new EnumMap<ConstraintType, boolean[]>(ConstraintType.class);

   private final MomentumBasedController momentumBasedController;

   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;

   private final BooleanYoVariable doFancyOnToesControl;

   private final HoldPositionState holdPositionState;
   private final SwingStateInterface swingState;
   private final MoveStraightState moveStraightState;
   private final TouchdownState touchdownOnToesState;
   private final TouchdownState touchdownOnHeelState;
   private final OnToesState onToesState;
   private final FullyConstrainedState supportState;

   private final FootSwitchInterface footSwitch;
   private final DoubleYoVariable footLoadThresholdToHoldPosition;

   private final FootControlHelper footControlHelper;

   private final BooleanYoVariable waitSingularityEscapeBeforeTransitionToNextState;

   public FootControlModule(RobotSide robotSide, WalkingControllerParameters walkingControllerParameters, YoSE3PIDGains swingFootControlGains,
         YoSE3PIDGains holdPositionFootControlGains, YoSE3PIDGains toeOffFootControlGains, YoSE3PIDGains edgeTouchdownFootControlGains,
         YoSE3PIDGains supportFootControlGains, DoubleProvider swingTimeProvider, MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      contactableFoot = momentumBasedController.getContactableFeet().get(robotSide);
      momentumBasedController.setPlaneContactCoefficientOfFriction(contactableFoot, coefficientOfFriction);
      momentumBasedController.setPlaneContactStateFullyConstrained(contactableFoot);

      RigidBody foot = contactableFoot.getRigidBody();
      String namePrefix = foot.getName();
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      footControlHelper = new FootControlHelper(robotSide, walkingControllerParameters, momentumBasedController, registry);

      this.momentumBasedController = momentumBasedController;

      footSwitch = momentumBasedController.getFootSwitches().get(robotSide);
      footLoadThresholdToHoldPosition = new DoubleYoVariable("footLoadThresholdToHoldPosition", registry);
      footLoadThresholdToHoldPosition.set(0.2);

      doFancyOnToesControl = new BooleanYoVariable(contactableFoot.getName() + "DoFancyOnToesControl", registry);
      doFancyOnToesControl.set(walkingControllerParameters.doFancyOnToesControl());

      waitSingularityEscapeBeforeTransitionToNextState = new BooleanYoVariable(namePrefix + "WaitSingularityEscapeBeforeTransitionToNextState", registry);

      legSingularityAndKneeCollapseAvoidanceControlModule = footControlHelper.getLegSingularityAndKneeCollapseAvoidanceControlModule();

      // set up states and state machine
      DoubleYoVariable time = momentumBasedController.getYoTime();
      stateMachine = new StateMachine<ConstraintType>(namePrefix + "State", namePrefix + "SwitchTime", ConstraintType.class, time, registry);
      setupContactStatesMap();

      YoVelocityProvider touchdownVelocityProvider = new YoVelocityProvider(namePrefix + "TouchdownVelocity", ReferenceFrame.getWorldFrame(), registry);
      touchdownVelocityProvider.set(new Vector3d(0.0, 0.0, walkingControllerParameters.getDesiredTouchdownVelocity()));

      YoVelocityProvider touchdownAccelerationProvider = new YoVelocityProvider(namePrefix + "TouchdownAcceleration", ReferenceFrame.getWorldFrame(), registry);
      touchdownAccelerationProvider.set(new Vector3d(0.0, 0.0, walkingControllerParameters.getDesiredTouchdownAcceleration()));

      List<AbstractFootControlState> states = new ArrayList<AbstractFootControlState>();
      touchdownOnToesState = new TouchdownState(ConstraintType.TOES_TOUCHDOWN, footControlHelper, touchdownVelocityProvider, edgeTouchdownFootControlGains, registry);
      states.add(touchdownOnToesState);

      touchdownOnHeelState = new TouchdownState(ConstraintType.HEEL_TOUCHDOWN, footControlHelper, touchdownVelocityProvider, edgeTouchdownFootControlGains, registry);
      states.add(touchdownOnHeelState);

      onToesState = new OnToesState(footControlHelper, toeOffFootControlGains, registry);
      states.add(onToesState);

      supportState = new FullyConstrainedState(footControlHelper, supportFootControlGains, registry);
      states.add(supportState);

      holdPositionState = new HoldPositionState(footControlHelper, holdPositionFootControlGains, registry);
      states.add(holdPositionState);

      if (USE_HEURISTIC_SWING_STATE)
      {
         HeuristicSwingState swingState = new HeuristicSwingState(footControlHelper, swingTimeProvider, swingFootControlGains, registry);
         states.add(swingState);
         this.swingState = swingState;
      }
      else
      {
         SwingState swingState = new SwingState(footControlHelper, swingTimeProvider, touchdownVelocityProvider, touchdownAccelerationProvider, swingFootControlGains, registry);
         states.add(swingState);
         this.swingState = swingState;
      }

      moveStraightState = new MoveStraightState(footControlHelper, swingFootControlGains, registry);
      states.add(moveStraightState);

      setupStateMachine(states);
   }

   private void setupContactStatesMap()
   {
      boolean[] falses = new boolean[contactableFoot.getTotalNumberOfContactPoints()];
      Arrays.fill(falses, false);
      boolean[] trues = new boolean[contactableFoot.getTotalNumberOfContactPoints()];
      Arrays.fill(trues, true);

      contactStatesMap.put(ConstraintType.SWING, falses);
      contactStatesMap.put(ConstraintType.MOVE_STRAIGHT, falses);
      contactStatesMap.put(ConstraintType.FULL, trues);
      contactStatesMap.put(ConstraintType.HOLD_POSITION, trues);
      contactStatesMap.put(ConstraintType.HEEL_TOUCHDOWN, getOnEdgeContactPointStates(contactableFoot, ConstraintType.HEEL_TOUCHDOWN));
      contactStatesMap.put(ConstraintType.TOES, getOnEdgeContactPointStates(contactableFoot, ConstraintType.TOES));
      contactStatesMap.put(ConstraintType.TOES_TOUCHDOWN, contactStatesMap.get(ConstraintType.TOES));
   }

   private void setupStateMachine(List<AbstractFootControlState> states)
   {
      // TODO Clean that up (Sylvain)
      FootStateTransitionAction footStateTransitionAction = new FootStateTransitionAction(footControlHelper, waitSingularityEscapeBeforeTransitionToNextState);
      for (AbstractFootControlState state : states)
      {
         for (AbstractFootControlState stateToTransitionTo : states)
         {
            FootStateTransitionCondition footStateTransitionCondition = new FootStateTransitionCondition(stateToTransitionTo, footControlHelper, waitSingularityEscapeBeforeTransitionToNextState);
            ConstraintType nextStateEnum = stateToTransitionTo.getStateEnum();
            state.addStateTransition(new StateTransition<ConstraintType>(nextStateEnum, footStateTransitionCondition, footStateTransitionAction));
         }
      }

      supportState.addStateTransition(new StateTransition<FootControlModule.ConstraintType>(ConstraintType.HOLD_POSITION, new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            if (isFootBarelyLoaded())
               return true;
            if (!doFancyOnToesControl.getBooleanValue())
               return false;
            return footControlHelper.isCoPOnEdge();
         }
      }, footStateTransitionAction));

      holdPositionState.addStateTransition(new StateTransition<FootControlModule.ConstraintType>(ConstraintType.FULL, new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            if (isFootBarelyLoaded())
               return false;
            return !footControlHelper.isCoPOnEdge();
         }
      }, footStateTransitionAction));

      for (State<ConstraintType> state : states)
      {
         stateMachine.addState(state);
      }
      stateMachine.setCurrentState(ConstraintType.FULL);
   }

   public void replanTrajectory(Footstep footstep)
   {
      swingState.replanTrajectory(footstep);
   }

   public void requestMoveStraightTouchdownForDisturbanceRecovery()
   {
      if (stateMachine.getCurrentState() != moveStraightState)
         return;
      moveStraightState.requestTouchdownForDisturbanceRecovery();
   }

   public void doSingularityEscape(boolean doSingularityEscape)
   {
      footControlHelper.doSingularityEscape(doSingularityEscape);
   }

   public void doSingularityEscape(double temporarySingularityEscapeNullspaceMultiplier)
   {
      footControlHelper.doSingularityEscape(temporarySingularityEscapeNullspaceMultiplier);
   }

   public void doSingularityEscapeBeforeTransitionToNextState()
   {
      doSingularityEscape(true);
      waitSingularityEscapeBeforeTransitionToNextState.set(true);
   }

   public double getJacobianDeterminant()
   {
      return footControlHelper.getJacobianDeterminant();
   }

   public boolean isInSingularityNeighborhood()
   {
      return footControlHelper.isJacobianDeterminantInRange();
   }

   public void setNullspaceMultiplier(double singularityEscapeNullspaceMultiplier)
   {
      footControlHelper.setNullspaceMultiplier(singularityEscapeNullspaceMultiplier);
   }

   public void setContactState(ConstraintType constraintType)
   {
      setContactState(constraintType, null);
   }

   public void setContactState(ConstraintType constraintType, FrameVector normalContactVector)
   {
      if (constraintType == ConstraintType.HOLD_POSITION || constraintType == ConstraintType.FULL)
      {
         if (constraintType == ConstraintType.HOLD_POSITION)
            System.out.println("Warning: HOLD_POSITION state is handled internally.");

         if (isFootBarelyLoaded())
            constraintType = ConstraintType.HOLD_POSITION;
         else
            constraintType = ConstraintType.FULL;

         footControlHelper.setFullyConstrainedNormalContactVector(normalContactVector);
      }

      momentumBasedController.setPlaneContactState(contactableFoot, contactStatesMap.get(constraintType), normalContactVector);

      if (getCurrentConstraintType() == constraintType) // Use resetCurrentState() for such case
         return;

      footControlHelper.requestState(constraintType);
   }

   private boolean isFootBarelyLoaded()
   {
      return footSwitch.computeFootLoadPercentage() < footLoadThresholdToHoldPosition.getDoubleValue();
   }

   public ConstraintType getCurrentConstraintType()
   {
      return stateMachine.getCurrentStateEnum();
   }

   public void doControl()
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.resetSwingParameters();
      footControlHelper.update();

      stateMachine.checkTransitionConditions();

      if (!isInFlatSupportState())
         footControlHelper.getPartialFootholdControlModule().reset();

      stateMachine.doAction();
   }

   // Used to restart the current state reseting the current state time
   public void resetCurrentState()
   {
      stateMachine.setCurrentState(getCurrentConstraintType());
   }

   public boolean isInFlatSupportState()
   {
      return getCurrentConstraintType() == ConstraintType.FULL || getCurrentConstraintType() == ConstraintType.HOLD_POSITION;
   }

   public boolean isInEdgeTouchdownState()
   {
      return getCurrentConstraintType() == ConstraintType.HEEL_TOUCHDOWN || getCurrentConstraintType() == ConstraintType.TOES_TOUCHDOWN;
   }

   private boolean[] getOnEdgeContactPointStates(ContactablePlaneBody contactableBody, ConstraintType constraintType)
   {
      FrameVector direction = new FrameVector(contactableBody.getFrameAfterParentJoint(), 1.0, 0.0, 0.0);
      if (constraintType == ConstraintType.HEEL_TOUCHDOWN)
         direction.scale(-1.0);

      int[] indexOfPointsInContact = DesiredFootstepCalculatorTools.findMaximumPointIndexesInDirection(contactableBody.getContactPointsCopy(), direction, 2);

      boolean[] contactPointStates = new boolean[contactableBody.getTotalNumberOfContactPoints()];

      for (int i = 0; i < indexOfPointsInContact.length; i++)
      {
         contactPointStates[indexOfPointsInContact[i]] = true;
      }

      return contactPointStates;
   }

   public void updateLegSingularityModule()
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.update();
   }

   public void correctCoMHeightTrajectoryForSingularityAvoidance(FrameVector2d comXYVelocity, CoMHeightTimeDerivativesData comHeightDataToCorrect,
         double zCurrent, ReferenceFrame pelvisZUpFrame)
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForSingularityAvoidance(comXYVelocity, comHeightDataToCorrect, zCurrent,
            pelvisZUpFrame, getCurrentConstraintType());
   }

   public void correctCoMHeightTrajectoryForCollapseAvoidance(FrameVector2d comXYVelocity, CoMHeightTimeDerivativesData comHeightDataToCorrect,
         double zCurrent, ReferenceFrame pelvisZUpFrame, double footLoadPercentage)
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForCollapseAvoidance(comXYVelocity, comHeightDataToCorrect, zCurrent,
            pelvisZUpFrame, footLoadPercentage, getCurrentConstraintType());
   }

   public void correctCoMHeightTrajectoryForUnreachableFootStep(CoMHeightTimeDerivativesData comHeightDataToCorrect)
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForUnreachableFootStep(comHeightDataToCorrect, getCurrentConstraintType());
   }

   public void setFootstep(Footstep footstep)
   {
      // TODO Used to pass the desireds from the toe off state to swing state. Clean that up.
      if (stateMachine.getCurrentStateEnum() == ConstraintType.TOES)
      {
         FrameOrientation initialOrientation = new FrameOrientation();
         FrameVector initialAngularVelocity = new FrameVector();
         onToesState.getDesireds(initialOrientation, initialAngularVelocity);
         swingState.setInitialDesireds(initialOrientation, initialAngularVelocity);
      }
      swingState.setFootstep(footstep);
      footControlHelper.updateWithPredictedContactPoints(footstep);
   }

   public boolean isLegDoingToeOffAndAtLimit()
   {
      if (getCurrentConstraintType() != ConstraintType.TOES)
         return false;
      RobotSide robotSide = footControlHelper.getRobotSide();
      OneDoFJoint kneeJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE);
      OneDoFJoint anklePitchJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
      double straightKneeThresholdInToeOFf = 0.42;
      boolean isKneeAlmostStraight = kneeJoint.getQ() < straightKneeThresholdInToeOFf;
      boolean isAnkleAtLowerLimit = anklePitchJoint.getQ() > anklePitchJoint.getJointLimitUpper() - 0.05;

      return isKneeAlmostStraight && isAnkleAtLowerLimit;
   }

   public void setFootPose(FramePose footPose, double trajectoryTime)
   {
      moveStraightState.setFootPose(footPose, trajectoryTime);
   }

   public double getHeelTouchdownInitialAngle()
   {
      return touchdownOnHeelState.getTouchdownInitialPitchAngle();
   }

   public double getToeTouchdownInitialAngle()
   {
      return touchdownOnToesState.getTouchdownInitialPitchAngle();
   }

   public void setPredictedToeOffDuration(double predictedToeOffDuration)
   {
      onToesState.setPredictedToeOffDuration(predictedToeOffDuration);
   }

   public void enableAnkleLimitAvoidanceInSupportState(boolean enable)
   {
      footControlHelper.enableAnkleLimitAvoidanceInSupportState(enable);
   }

   public void resetHeightCorrectionParametersForSingularityAvoidance()
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.resetHeightCorrectionParameters();
   }

   public void requestSwingSpeedUp(double speedUpFactor)
   {
      swingState.requestSwingSpeedUp(speedUpFactor);
   }
}