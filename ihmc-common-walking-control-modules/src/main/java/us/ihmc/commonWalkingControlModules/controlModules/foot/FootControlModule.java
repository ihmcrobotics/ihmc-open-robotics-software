package us.ihmc.commonWalkingControlModules.controlModules.foot;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType.GEQ_INEQUALITY;
import static us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType.LEQ_INEQUALITY;

import java.util.Arrays;
import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.configurations.AnkleIKSolver;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.dataStructures.parameters.FrameParameterVector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FootControlModule
{
   private final YoVariableRegistry registry;
   private final ContactablePlaneBody contactableFoot;

   public enum ConstraintType
   {
      FULL, TOES, SWING, MOVE_VIA_WAYPOINTS, TOUCHDOWN;

      public boolean isLoadBearing()
      {
         switch (this)
         {
         case FULL:
         case TOES:
            return true;
         default:
            return false;
         }
      }
   }

   private static final double coefficientOfFriction = 0.5;

   private final StateMachine<ConstraintType, AbstractFootControlState> stateMachine;
   private final YoEnum<ConstraintType> requestedState;
   private final EnumMap<ConstraintType, boolean[]> contactStatesMap = new EnumMap<ConstraintType, boolean[]>(ConstraintType.class);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final RobotSide robotSide;

   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;

   private final SwingState swingState;
   private final MoveViaWaypointsState moveViaWaypointsState;
   private final TouchDownState touchdownState;
   private final OnToesState onToesState;
   private final SupportState supportState;

   private final YoDouble footLoadThresholdToHoldPosition;

   private final FootControlHelper footControlHelper;

   private final YoBoolean requestExploration;
   private final YoBoolean resetFootPolygon;
   
   private final FramePose3D desiredPose = new FramePose3D();
   private final FrameVector3D desiredLinearVelocity = new FrameVector3D();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final UnloadedAnkleControlModule ankleControlModule;

   private final DoubleProvider maxWeightFractionPerFoot;
   private final DoubleProvider minWeightFractionPerFoot;
   private final YoDouble minZForce;
   private final YoDouble maxZForce;
   private final double robotWeightFz;
   private final ContactWrenchCommand maxWrenchCommand;
   private final ContactWrenchCommand minWrenchCommand;

   public FootControlModule(RobotSide robotSide, ToeOffCalculator toeOffCalculator, WalkingControllerParameters walkingControllerParameters,
                            PIDSE3GainsReadOnly swingFootControlGains, PIDSE3GainsReadOnly holdPositionFootControlGains,
                            PIDSE3GainsReadOnly toeOffFootControlGains, HighLevelHumanoidControllerToolbox controllerToolbox,
                            ExplorationParameters explorationParameters, DoubleProvider minWeightFractionPerFoot, DoubleProvider maxWeightFractionPerFoot,
                            YoVariableRegistry parentRegistry)
   {
      contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      controllerToolbox.setFootContactCoefficientOfFriction(robotSide, coefficientOfFriction);
      controllerToolbox.setFootContactStateFullyConstrained(robotSide);

      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Foot";
      registry = new YoVariableRegistry(sidePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      footControlHelper = new FootControlHelper(robotSide, walkingControllerParameters, controllerToolbox, explorationParameters, registry);

      this.controllerToolbox = controllerToolbox;
      this.robotSide = robotSide;

      footLoadThresholdToHoldPosition = new YoDouble("footLoadThresholdToHoldPosition", registry);
      footLoadThresholdToHoldPosition.set(0.2);
      
      legSingularityAndKneeCollapseAvoidanceControlModule = footControlHelper.getLegSingularityAndKneeCollapseAvoidanceControlModule();

      requestedState = YoEnum.create(namePrefix + "RequestedState", "", ConstraintType.class, registry, true);
      requestedState.set(null);

      setupContactStatesMap();

      Vector3D defaultTouchdownVelocity = new Vector3D(0.0, 0.0, swingTrajectoryParameters.getDesiredTouchdownVelocity());
      FrameParameterVector3D touchdownVelocity = new FrameParameterVector3D(namePrefix + "TouchdownVelocity", ReferenceFrame.getWorldFrame(),
                                                                            defaultTouchdownVelocity, registry);

      Vector3D defaultTouchdownAcceleration = new Vector3D(0.0, 0.0, swingTrajectoryParameters.getDesiredTouchdownAcceleration());
      FrameParameterVector3D touchdownAcceleration = new FrameParameterVector3D(namePrefix + "TouchdownAcceleration", ReferenceFrame.getWorldFrame(),
                                                                                defaultTouchdownAcceleration, registry);

      onToesState = new OnToesState(footControlHelper, toeOffCalculator, toeOffFootControlGains, registry);
      supportState = new SupportState(footControlHelper, holdPositionFootControlGains, registry);
      swingState = new SwingState(footControlHelper, touchdownVelocity, touchdownAcceleration, swingFootControlGains, registry);
      touchdownState = new TouchDownState(footControlHelper, swingFootControlGains, registry);
      moveViaWaypointsState = new MoveViaWaypointsState(footControlHelper, touchdownVelocity, touchdownAcceleration, swingFootControlGains, registry);

      stateMachine = setupStateMachine(namePrefix);

      requestExploration = new YoBoolean(namePrefix + "RequestExploration", registry);
      resetFootPolygon = new YoBoolean(namePrefix + "ResetFootPolygon", registry);

      AnkleIKSolver ankleIKSolver = walkingControllerParameters.getAnkleIKSolver();
      if (ankleIKSolver != null)
      {
         FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
         ankleControlModule = new UnloadedAnkleControlModule(fullRobotModel, robotSide, ankleIKSolver, registry);
      }
      else
      {
         ankleControlModule = null;
      }

      this.maxWeightFractionPerFoot = maxWeightFractionPerFoot;
      this.minWeightFractionPerFoot = minWeightFractionPerFoot;
      robotWeightFz = controllerToolbox.getFullRobotModel().getTotalMass() * controllerToolbox.getGravityZ();

      if (minWeightFractionPerFoot != null && maxWeightFractionPerFoot != null)
      {
         maxWrenchCommand = new ContactWrenchCommand(LEQ_INEQUALITY);
         minWrenchCommand = new ContactWrenchCommand(GEQ_INEQUALITY);
         setupWrenchCommand(maxWrenchCommand);
         setupWrenchCommand(minWrenchCommand);
         minZForce = new YoDouble(robotSide.getLowerCaseName() + "MinZForce", registry);
         maxZForce = new YoDouble(robotSide.getLowerCaseName() + "MaxZForce", registry);
      }
      else
      {
         maxWrenchCommand = null;
         minWrenchCommand = null;
         minZForce = null;
         maxZForce = null;
      }
   }

   private void setupWrenchCommand(ContactWrenchCommand command)
   {
      command.setRigidBody(contactableFoot.getRigidBody());
      command.getSelectionMatrix().clearSelection();
      command.getSelectionMatrix().setSelectionFrame(ReferenceFrame.getWorldFrame());
      command.getSelectionMatrix().selectLinearZ(true);
      command.getWrench().setToZero(contactableFoot.getRigidBody().getBodyFixedFrame(), ReferenceFrame.getWorldFrame());
   }

   private void setupContactStatesMap()
   {
      boolean[] falses = new boolean[contactableFoot.getTotalNumberOfContactPoints()];
      Arrays.fill(falses, false);
      boolean[] trues = new boolean[contactableFoot.getTotalNumberOfContactPoints()];
      Arrays.fill(trues, true);

      contactStatesMap.put(ConstraintType.SWING, falses);
      contactStatesMap.put(ConstraintType.MOVE_VIA_WAYPOINTS, falses);
      contactStatesMap.put(ConstraintType.FULL, trues);
//      contactStatesMap.put(ConstraintType.TOES, getOnEdgeContactPointStates(contactableFoot, ConstraintType.TOES));
      contactStatesMap.put(ConstraintType.TOES, trues);
      contactStatesMap.put(ConstraintType.TOUCHDOWN, falses);
   }

   private StateMachine<ConstraintType, AbstractFootControlState> setupStateMachine(String namePrefix)
   {
      StateMachineFactory<ConstraintType, AbstractFootControlState> factory = new StateMachineFactory<>(ConstraintType.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(footControlHelper.getHighLevelHumanoidControllerToolbox().getYoTime());
      factory.addState(ConstraintType.TOES, onToesState);
      factory.addState(ConstraintType.FULL, supportState);
      factory.addState(ConstraintType.SWING, swingState);
      factory.addState(ConstraintType.TOUCHDOWN, touchdownState);
      factory.addState(ConstraintType.MOVE_VIA_WAYPOINTS, moveViaWaypointsState);
      
      for (ConstraintType from : ConstraintType.values())
      {
         factory.addRequestedTransition(from, requestedState);
         factory.addRequestedTransition(from, from, requestedState);
      }
      
      return factory.build(ConstraintType.FULL);
   }

   public void setWeights(Vector3DReadOnly loadedFootAngularWeight, Vector3DReadOnly loadedFootLinearWeight, Vector3DReadOnly footAngularWeight,
                          Vector3DReadOnly footLinearWeight)
   {
      swingState.setWeights(footAngularWeight, footLinearWeight);
      moveViaWaypointsState.setWeights(footAngularWeight, footLinearWeight);
      touchdownState.setWeights(footAngularWeight, footLinearWeight);
      onToesState.setWeights(loadedFootAngularWeight, loadedFootLinearWeight);
      supportState.setWeights(loadedFootAngularWeight, loadedFootLinearWeight);
   }

   public void setAdjustedFootstepAndTime(Footstep adjustedFootstep, double swingTime)
   {
      swingState.setAdjustedFootstepAndTime(adjustedFootstep, swingTime);
   }

   public void requestTouchdownForDisturbanceRecovery()
   {
      if (stateMachine.getCurrentState() == moveViaWaypointsState)
         moveViaWaypointsState.requestTouchdownForDisturbanceRecovery(stateMachine.getTimeInCurrentState());
   }
   
   public void requestTouchdown()
   {
    if(stateMachine.getCurrentStateKey() == ConstraintType.SWING)
    {
       setContactState(ConstraintType.TOUCHDOWN);
       swingState.getDesireds(desiredPose, desiredLinearVelocity, desiredAngularVelocity);
       touchdownState.initialize(desiredPose, desiredLinearVelocity, desiredAngularVelocity);
    }
   }

   public void requestStopTrajectoryIfPossible()
   {
      if (stateMachine.getCurrentState() == moveViaWaypointsState)
         moveViaWaypointsState.requestStopTrajectory();
   }

   public void setContactState(ConstraintType constraintType)
   {
      setContactState(constraintType, null);
   }

   public void setContactState(ConstraintType constraintType, FrameVector3D normalContactVector)
   {
      if (constraintType == ConstraintType.FULL)
      {
         footControlHelper.setFullyConstrainedNormalContactVector(normalContactVector);
      }

      controllerToolbox.setFootContactState(robotSide, contactStatesMap.get(constraintType), normalContactVector);

      if (getCurrentConstraintType() == constraintType) // Use resetCurrentState() for such case
         return;

      requestedState.set(constraintType);
   }

   public ConstraintType getCurrentConstraintType()
   {
      return stateMachine.getCurrentStateKey();
   }

   public void initialize()
   {
      stateMachine.resetToInitialState();
      resetLoadConstraints();
   }

   public void doControl()
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.resetSwingParameters();
      }

      footControlHelper.update();

      if (resetFootPolygon.getBooleanValue())
      {
         resetFootPolygon();
      }

      if (requestExploration.getBooleanValue())
      {
         requestExploration();
      }
      
      stateMachine.doTransitions();

      if (!isInFlatSupportState() && footControlHelper.getPartialFootholdControlModule() != null)
         footControlHelper.getPartialFootholdControlModule().reset();
      

      stateMachine.doAction();

      if (ankleControlModule != null)
      {
         ankleControlModule.compute(stateMachine.getCurrentStateKey(), stateMachine.getCurrentState());
      }
   }

   // Used to restart the current state reseting the current state time
   public void resetCurrentState()
   {
      stateMachine.resetCurrentState();
   }
   
   public boolean isInFlatSupportState()
   {
      ConstraintType currentConstraintType = getCurrentConstraintType();
      return currentConstraintType == ConstraintType.FULL;
   }

   public boolean isInToeOff()
   {
      return getCurrentConstraintType() == ConstraintType.TOES;
   }

   public void setUsePointContactInToeOff(boolean usePointContact)
   {
      onToesState.setUsePointContact(usePointContact);
   }

   private boolean[] getOnEdgeContactPointStates(ContactablePlaneBody contactableBody, ConstraintType constraintType)
   {
      FrameVector3D direction = new FrameVector3D(contactableBody.getFrameAfterParentJoint(), 1.0, 0.0, 0.0);

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
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.update();
      }
   }

   public void correctCoMHeightTrajectoryForSingularityAvoidance(FrameVector2D comXYVelocity, CoMHeightTimeDerivativesData comHeightDataToCorrect,
                                                                 double zCurrent, ReferenceFrame pelvisZUpFrame)
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForSingularityAvoidance(comXYVelocity, comHeightDataToCorrect, zCurrent,
                                                                                                               pelvisZUpFrame, getCurrentConstraintType());
      }
   }

   public void correctCoMHeightTrajectoryForCollapseAvoidance(FrameVector2D comXYVelocity, CoMHeightTimeDerivativesData comHeightDataToCorrect, double zCurrent,
                                                              ReferenceFrame pelvisZUpFrame, double footLoadPercentage)
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForCollapseAvoidance(comXYVelocity, comHeightDataToCorrect, zCurrent,
                                                                                                            pelvisZUpFrame, footLoadPercentage,
                                                                                                            getCurrentConstraintType());
      }
   }

   public void correctCoMHeightTrajectoryForUnreachableFootStep(CoMHeightTimeDerivativesData comHeightDataToCorrect)
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForUnreachableFootStep(comHeightDataToCorrect,
                                                                                                              getCurrentConstraintType());
      }
   }

   public void setFootstep(Footstep footstep, double swingTime, double touchdownTime)
   {
      swingState.setFootstep(footstep, swingTime);
      touchdownState.setTouchdownDuration(touchdownTime);
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command)
   {
      moveViaWaypointsState.handleFootTrajectoryCommand(command);
   }

   public void resetHeightCorrectionParametersForSingularityAvoidance()
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.resetHeightCorrectionParameters();
      }
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @param speedUpFactor
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(double speedUpFactor)
   {
      return swingState.requestSwingSpeedUp(speedUpFactor);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(stateMachine.getCurrentState().getInverseDynamicsCommand());
      if (ankleControlModule != null)
      {
         inverseDynamicsCommandList.addCommand(ankleControlModule.getInverseDynamicsCommand());
      }
      if (maxWrenchCommand != null && stateMachine.getCurrentStateKey().isLoadBearing())
      {
         inverseDynamicsCommandList.addCommand(maxWrenchCommand);
         inverseDynamicsCommandList.addCommand(minWrenchCommand);
      }
      return inverseDynamicsCommandList;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   public JointDesiredOutputListReadOnly getJointDesiredData()
   {
      if (ankleControlModule != null)
      {
         return ankleControlModule.getJointDesiredOutputList();
      }
      return null;
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (ConstraintType constraintType : ConstraintType.values())
      {
         AbstractFootControlState state = stateMachine.getState(constraintType);
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }
   
   public boolean isInTouchdown()
   {
      return getCurrentConstraintType().equals(ConstraintType.TOUCHDOWN);
   }

   public void initializeFootExploration()
   {
      supportState.requestFootholdExploration();
   }

   public boolean isFootToeingOffSlipping()
   {
      if (getCurrentConstraintType() != ConstraintType.TOES)
         return false;
      if (footControlHelper.getToeSlippingDetector() == null)
         return false;
      return footControlHelper.getToeSlippingDetector().isToeSlipping();
   }

   private void requestExploration()
   {
      if (!isInFlatSupportState()) return;
      requestExploration.set(false);
      initializeFootExploration();
   }

   private void resetFootPolygon()
   {
      if (!isInFlatSupportState()) return;
      resetFootPolygon.set(false);
      if (footControlHelper.getPartialFootholdControlModule() != null)
      {
         footControlHelper.getPartialFootholdControlModule().reset();
      }
      controllerToolbox.resetFootSupportPolygon(robotSide);
   }

   public void unload(double percentInUnloading)
   {
      if (minWrenchCommand == null)
         return;
      minZForce.set((1.0 - percentInUnloading) * minWeightFractionPerFoot.getValue() * robotWeightFz);
      maxZForce.set((1.0 - percentInUnloading) * maxWeightFractionPerFoot.getValue() * robotWeightFz);

      updateWrenchCommands();
   }

   public void resetLoadConstraints()
   {
      if (minWrenchCommand == null)
         return;
      minZForce.set(minWeightFractionPerFoot.getValue() * robotWeightFz);
      maxZForce.set(maxWeightFractionPerFoot.getValue() * robotWeightFz);

      updateWrenchCommands();
   }

   private void updateWrenchCommands()
   {
      // Make sure the max force is always a little larger then the min force. This is to avoid sending conflicting constraints.
      maxZForce.set(Math.max(maxZForce.getValue(), minZForce.getValue() + 1.0E-5));

      minWrenchCommand.getWrench().setLinearPartZ(minZForce.getValue());
      maxWrenchCommand.getWrench().setLinearPartZ(maxZForce.getValue());
   }
}