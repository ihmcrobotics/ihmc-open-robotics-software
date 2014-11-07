package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.TrajectoryParameters;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVelocityProvider;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateTransition;

public class FootControlModule
{
   public static final boolean USE_AUTOMATIC_FOOT_SHRINK = true;
   public static final boolean USE_SUPPORT_FOOT_HOLD_POSITION_STATE = true;

   private final YoVariableRegistry registry;
   private final ContactablePlaneBody contactableFoot;

   public enum ConstraintType
   {
      FULL, HOLD_POSITION, HEEL_TOUCHDOWN, TOES_TOUCHDOWN, TOES, SWING, MOVE_STRAIGHT
   }

   private static final double coefficientOfFriction = 0.8;

   private final StateMachine<ConstraintType> stateMachine;
   private final EnumYoVariable<ConstraintType> requestedState;
   private final EnumMap<ConstraintType, boolean[]> contactStatesMap = new EnumMap<ConstraintType, boolean[]>(ConstraintType.class);

   private final RigidBodySpatialAccelerationControlModule accelerationControlModule;
   private final MomentumBasedController momentumBasedController;

   private final DoubleYoVariable jacobianDeterminant;
   private final BooleanYoVariable jacobianDeterminantInRange;

   private final BooleanYoVariable doSingularityEscape;
   private final BooleanYoVariable waitSingularityEscapeBeforeTransitionToNextState;
   private final DoubleYoVariable singularityEscapeNullspaceMultiplier;
   private final DoubleYoVariable nullspaceMultiplier;
   private final GeometricJacobian jacobian;

   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;

   private final BooleanYoVariable requestHoldPosition;
   private final FrameVector fullyConstrainedNormalContactVector;

   private final BooleanYoVariable doFancyOnToesControl;

   private final HoldPositionState holdPositionState;
   private final SwingState swingState;
   private final MoveStraightState moveStraightState;
   private final TouchdownState touchdownOnToesState;
   private final TouchdownState touchdownOnHeelState;
   private final OnToesState onToesState;

   private final FootSwitchInterface footSwitch;
   private final DoubleYoVariable footLoadThresholdToHoldPosition;
   
   private final PartialFootholdControlModule partialFootholdControlModule;

   public FootControlModule(RobotSide robotSide, WalkingControllerParameters walkingControllerParameters, YoSE3PIDGains swingFootControlGains,
         YoSE3PIDGains holdPositionFootControlGains, YoSE3PIDGains toeOffFootControlGains, YoSE3PIDGains supportFootControlGains, DoubleProvider swingTimeProvider,
         MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      contactableFoot = momentumBasedController.getContactableFeet().get(robotSide);
      momentumBasedController.setPlaneContactCoefficientOfFriction(contactableFoot, coefficientOfFriction);
      momentumBasedController.setPlaneContactStateFullyConstrained(contactableFoot);

      RigidBody foot = contactableFoot.getRigidBody();
      String namePrefix = foot.getName();
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();
      double controlDT = momentumBasedController.getControlDT();
      partialFootholdControlModule = new PartialFootholdControlModule(namePrefix, controlDT, contactableFoot, twistCalculator, walkingControllerParameters, registry, momentumBasedController.getDynamicGraphicObjectsListRegistry());
      
      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(fullRobotModel.getPelvis(), foot, foot.getBodyFixedFrame());

      this.jacobian = momentumBasedController.getJacobian(jacobianId);
      if (foot != jacobian.getEndEffector())
         throw new RuntimeException("contactablePlaneBody does not match jacobian end effector!");

      this.requestedState = EnumYoVariable.create(namePrefix + "RequestedState", "", ConstraintType.class, registry, true);
      this.momentumBasedController = momentumBasedController;

      this.requestHoldPosition = new BooleanYoVariable(namePrefix + "RequestedHoldPosition", registry);
      footSwitch = momentumBasedController.getFootSwitches().get(robotSide);
      footLoadThresholdToHoldPosition = new DoubleYoVariable("footLoadThresholdToHoldPosition", registry);
      footLoadThresholdToHoldPosition.set(0.2);

      fullyConstrainedNormalContactVector = new FrameVector(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);

      ReferenceFrame bodyFrame = contactableFoot.getFrameAfterParentJoint();
      accelerationControlModule = new RigidBodySpatialAccelerationControlModule(namePrefix, twistCalculator, foot, bodyFrame,
            momentumBasedController.getControlDT(), registry);
      doSingularityEscape = new BooleanYoVariable(namePrefix + "DoSingularityEscape", registry);
      waitSingularityEscapeBeforeTransitionToNextState = new BooleanYoVariable(namePrefix + "WaitSingularityEscapeBeforeTransitionToNextState", registry);
      jacobianDeterminant = new DoubleYoVariable(namePrefix + "JacobianDeterminant", registry);
      jacobianDeterminantInRange = new BooleanYoVariable(namePrefix + "JacobianDeterminantInRange", registry);
      nullspaceMultiplier = new DoubleYoVariable(namePrefix + "NullspaceMultiplier", registry);
      //      nullspaceCalculator = new NullspaceCalculator(jacobian.getNumberOfColumns(), true);
      singularityEscapeNullspaceMultiplier = new DoubleYoVariable(namePrefix + "SingularityEscapeNullspaceMultiplier", registry);

      doFancyOnToesControl = new BooleanYoVariable(contactableFoot.getName() + "DoFancyOnToesControl", registry);
      doFancyOnToesControl.set(walkingControllerParameters.doFancyOnToesControl());

      legSingularityAndKneeCollapseAvoidanceControlModule = new LegSingularityAndKneeCollapseAvoidanceControlModule(namePrefix, contactableFoot, robotSide,
            walkingControllerParameters, momentumBasedController, registry);

      // set up states and state machine
      DoubleYoVariable time = momentumBasedController.getYoTime();
      stateMachine = new StateMachine<ConstraintType>(namePrefix + "State", namePrefix + "SwitchTime", ConstraintType.class, time, registry);
      setupContactStatesMap();

      YoVelocityProvider touchdownVelocityProvider = new YoVelocityProvider(namePrefix + "TouchdownVelocity", ReferenceFrame.getWorldFrame(), registry);
      touchdownVelocityProvider.set(new Vector3d(0.0, 0.0, walkingControllerParameters.getDesiredTouchdownVelocity()));

      List<AbstractFootControlState> states = new ArrayList<AbstractFootControlState>();
      touchdownOnToesState = new TouchdownState(ConstraintType.TOES_TOUCHDOWN, walkingControllerParameters, touchdownVelocityProvider,
            accelerationControlModule, momentumBasedController, contactableFoot, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange,
            doSingularityEscape, robotSide, registry);
      states.add(touchdownOnToesState);

      touchdownOnHeelState = new TouchdownState(ConstraintType.HEEL_TOUCHDOWN, walkingControllerParameters, touchdownVelocityProvider,
            accelerationControlModule, momentumBasedController, contactableFoot, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange,
            doSingularityEscape, robotSide, registry);
      states.add(touchdownOnHeelState);

      onToesState = new OnToesState(walkingControllerParameters, accelerationControlModule, momentumBasedController, contactableFoot, jacobianId,
            nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape, toeOffFootControlGains, robotSide, registry);
      states.add(onToesState);

      FullyConstrainedState supportState = new FullyConstrainedState(accelerationControlModule, momentumBasedController, contactableFoot, requestHoldPosition,
            requestedState, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape, partialFootholdControlModule,
            fullyConstrainedNormalContactVector, doFancyOnToesControl, supportFootControlGains, robotSide, registry);
      states.add(supportState);

      holdPositionState = new HoldPositionState(accelerationControlModule, momentumBasedController, contactableFoot, requestHoldPosition, requestedState,
            jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape, partialFootholdControlModule, fullyConstrainedNormalContactVector,
            holdPositionFootControlGains, robotSide, registry);
      states.add(holdPositionState);

      swingState = new SwingState(swingTimeProvider, touchdownVelocityProvider, accelerationControlModule, momentumBasedController, contactableFoot,
            jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape, legSingularityAndKneeCollapseAvoidanceControlModule,
            swingFootControlGains, robotSide, registry, walkingControllerParameters);
      states.add(swingState);

      moveStraightState = new MoveStraightState(swingTimeProvider, accelerationControlModule, momentumBasedController, contactableFoot, jacobianId,
            nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape, legSingularityAndKneeCollapseAvoidanceControlModule, swingFootControlGains,
            robotSide, registry);
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
      contactStatesMap.put(ConstraintType.TOES, trues);
      contactStatesMap.put(ConstraintType.TOES_TOUCHDOWN, contactStatesMap.get(ConstraintType.TOES));
   }

   private void setupStateMachine(List<AbstractFootControlState> states)
   {
      // TODO Clean that up (Sylvain)
      for (AbstractFootControlState state : states)
      {
         for (AbstractFootControlState stateToTransitionTo : states)
         {
            FootStateTransitionCondition footStateTransitionCondition = new FootStateTransitionCondition(stateToTransitionTo, jacobian, requestedState,
                  doSingularityEscape, jacobianDeterminantInRange, waitSingularityEscapeBeforeTransitionToNextState);
            state.addStateTransition(new StateTransition<ConstraintType>(stateToTransitionTo.getStateEnum(), footStateTransitionCondition,
                  new FootStateTransitionAction(requestedState, doSingularityEscape, waitSingularityEscapeBeforeTransitionToNextState)));
         }
      }

      for (State<ConstraintType> state : states)
      {
         stateMachine.addState(state);
      }
      stateMachine.setCurrentState(ConstraintType.FULL);
   }

   public void replanTrajectory(Footstep footstep, double swingTimeRemaining, boolean useLowHeightTrajectory)
   {
      swingState.replanTrajectory(footstep, swingTimeRemaining, useLowHeightTrajectory);
   }

   public void doSingularityEscape(boolean doSingularityEscape)
   {
      this.doSingularityEscape.set(doSingularityEscape);
      this.nullspaceMultiplier.set(singularityEscapeNullspaceMultiplier.getDoubleValue());
   }

   public void doSingularityEscape(double temporarySingularityEscapeNullspaceMultiplier)
   {
      doSingularityEscape.set(true);
      this.nullspaceMultiplier.set(temporarySingularityEscapeNullspaceMultiplier);
   }

   public void doSingularityEscapeBeforeTransitionToNextState()
   {
      doSingularityEscape(true);
      waitSingularityEscapeBeforeTransitionToNextState.set(true);
   }

   public double getJacobianDeterminant()
   {
      return jacobianDeterminant.getDoubleValue();
   }

   public boolean isInSingularityNeighborhood()
   {
      return jacobianDeterminantInRange.getBooleanValue();
   }

   public void setParameters(double singularityEscapeNullspaceMultiplier)
   {
      this.singularityEscapeNullspaceMultiplier.set(singularityEscapeNullspaceMultiplier);
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

         if (USE_SUPPORT_FOOT_HOLD_POSITION_STATE)
         {
            if (requestHoldPosition.getBooleanValue())
               constraintType = ConstraintType.HOLD_POSITION;
            else
               constraintType = ConstraintType.FULL;
         }
         else
            constraintType = ConstraintType.FULL;

         if (normalContactVector != null)
            fullyConstrainedNormalContactVector.setIncludingFrame(normalContactVector);
         else
            fullyConstrainedNormalContactVector.setIncludingFrame(contactableFoot.getSoleFrame(), 0.0, 0.0, 1.0);
      }

      momentumBasedController.setPlaneContactState(contactableFoot, contactStatesMap.get(constraintType), normalContactVector);

      if (getCurrentConstraintType() == constraintType) // Use resetCurrentState() for such case
         return;

      requestedState.set(constraintType);
   }

   public ConstraintType getCurrentConstraintType()
   {
      return stateMachine.getCurrentStateEnum();
   }

   public void doControl()
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.resetSwingParameters();
      if (USE_SUPPORT_FOOT_HOLD_POSITION_STATE)
         requestHoldPosition.set(footSwitch.computeFootLoadPercentage() < footLoadThresholdToHoldPosition.getDoubleValue());
      jacobianDeterminant.set(jacobian.det());

      stateMachine.checkTransitionConditions();

      if (!isInFlatSupportState())
         partialFootholdControlModule.reset();

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

   public void setFootstep(Footstep footstep, TrajectoryParameters trajectoryParameters)
   {
      swingState.setFootstep(footstep, trajectoryParameters, false);
   }

   public void setFootPose(FramePose footPose)
   {
      moveStraightState.setFootPose(footPose);
   }

   public double getHeelTouchdownInitialAngle()
   {
      return touchdownOnHeelState.getTouchdownInitialPitchAngle();
   }

   public double getToeTouchdownInitialAngle()
   {
      return touchdownOnToesState.getTouchdownInitialPitchAngle();
   }
}