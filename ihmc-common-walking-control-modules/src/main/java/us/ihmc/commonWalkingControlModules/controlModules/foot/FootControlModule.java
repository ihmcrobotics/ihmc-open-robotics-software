package us.ihmc.commonWalkingControlModules.controlModules.foot;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType.GEQ_INEQUALITY;
import static us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType.LEQ_INEQUALITY;

import java.util.Arrays;
import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.YoSwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.controlModules.SwingTrajectoryCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold.YoPartialFootholdModuleParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.heightPlanning.CoMHeightTimeDerivativesDataBasics;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FootControlModule
{
   private final YoRegistry registry;
   private final ContactablePlaneBody contactableFoot;

   public enum ConstraintType
   {
      FULL, TOES, SWING, MOVE_VIA_WAYPOINTS;

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

   private static final double defaultCoefficientOfFriction = 0.8;
   private final DoubleParameter coefficientOfFriction;

   private final StateMachine<ConstraintType, AbstractFootControlState> stateMachine;
   private final YoEnum<ConstraintType> requestedState;
   private final EnumMap<ConstraintType, boolean[]> contactStatesMap = new EnumMap<ConstraintType, boolean[]>(ConstraintType.class);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final RobotSide robotSide;

   private final WorkspaceLimiterControlModule workspaceLimiterControlModule;

   private final SwingState swingState;
   private final MoveViaWaypointsState moveViaWaypointsState;
   private final OnToesState onToesState;
   private final SupportState supportState;

   private final YoDouble footLoadThresholdToHoldPosition;

   private final FootControlHelper footControlHelper;

   private final YoBoolean requestExploration;
   private final YoBoolean resetFootPolygon;

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();

   private final DoubleProvider maxWeightFractionPerFoot;
   private final DoubleProvider minWeightFractionPerFoot;
   private final YoDouble minZForce;
   private final YoDouble maxZForce;
   private final double robotWeightFz;
   private final ContactWrenchCommand maxWrenchCommand;
   private final ContactWrenchCommand minWrenchCommand;
   private final int numberOfBasisVectors;

   public FootControlModule(RobotSide robotSide,
                            ToeOffCalculator toeOffCalculator,
                            WalkingControllerParameters walkingControllerParameters,
                            YoSwingTrajectoryParameters swingTrajectoryParameters,
                            WorkspaceLimiterParameters workspaceLimiterParameters,
                            PIDSE3GainsReadOnly swingFootControlGains,
                            PIDSE3GainsReadOnly holdPositionFootControlGains,
                            PIDSE3GainsReadOnly toeOffFootControlGains,
                            HighLevelHumanoidControllerToolbox controllerToolbox,
                            ExplorationParameters explorationParameters,
                            YoPartialFootholdModuleParameters footholdRotationParameters,
                            SupportStateParameters supportStateParameters,
                            DoubleProvider minWeightFractionPerFoot,
                            DoubleProvider maxWeightFractionPerFoot,
                            YoRegistry parentRegistry)
   {
      contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      controllerToolbox.setFootContactStateFullyConstrained(robotSide);

      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Foot";
      registry = new YoRegistry(sidePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      footControlHelper = new FootControlHelper(robotSide,
                                                walkingControllerParameters,
                                                swingTrajectoryParameters,
                                                workspaceLimiterParameters,
                                                controllerToolbox,
                                                explorationParameters,
                                                footholdRotationParameters,
                                                supportStateParameters,
                                                registry);

      this.controllerToolbox = controllerToolbox;
      this.robotSide = robotSide;

      footLoadThresholdToHoldPosition = new YoDouble("footLoadThresholdToHoldPosition", registry);
      footLoadThresholdToHoldPosition.set(0.2);

      workspaceLimiterControlModule = footControlHelper.getWorkspaceLimiterControlModule();

      requestedState = new YoEnum<>(namePrefix + "RequestedState", "", registry, ConstraintType.class, true);
      requestedState.set(null);

      setupContactStatesMap();

      onToesState = new OnToesState(footControlHelper, toeOffCalculator, toeOffFootControlGains, registry);
      supportState = new SupportState(footControlHelper, holdPositionFootControlGains, registry);
      swingState = new SwingState(footControlHelper, swingFootControlGains, registry);
      moveViaWaypointsState = new MoveViaWaypointsState(footControlHelper, swingFootControlGains, registry);

      stateMachine = setupStateMachine(namePrefix);

      requestExploration = new YoBoolean(namePrefix + "RequestExploration", registry);
      resetFootPolygon = new YoBoolean(namePrefix + "ResetFootPolygon", registry);

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

      numberOfBasisVectors = walkingControllerParameters.getMomentumOptimizationSettings().getNumberOfBasisVectorsPerContactPoint();

      String targetRegistryName = FeetManager.class.getSimpleName();
      String parameterRegistryName = FootControlModule.class.getSimpleName() + "Parameters";
      coefficientOfFriction = ParameterProvider.getOrCreateParameter(targetRegistryName,
                                                                     parameterRegistryName,
                                                                     "CoefficientOfFriction",
                                                                     registry,
                                                                     defaultCoefficientOfFriction);
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
   }

   private StateMachine<ConstraintType, AbstractFootControlState> setupStateMachine(String namePrefix)
   {
      StateMachineFactory<ConstraintType, AbstractFootControlState> factory = new StateMachineFactory<>(ConstraintType.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(footControlHelper.getHighLevelHumanoidControllerToolbox().getYoTime());
      factory.addState(ConstraintType.TOES, onToesState);
      factory.addState(ConstraintType.FULL, supportState);
      factory.addState(ConstraintType.SWING, swingState);
      factory.addState(ConstraintType.MOVE_VIA_WAYPOINTS, moveViaWaypointsState);

      for (ConstraintType from : ConstraintType.values())
      {
         factory.addRequestedTransition(from, requestedState);
         factory.addRequestedTransition(from, from, requestedState);
      }

      return factory.build(ConstraintType.FULL);
   }

   public void setWeights(Vector3DReadOnly loadedFootAngularWeight,
                          Vector3DReadOnly loadedFootLinearWeight,
                          Vector3DReadOnly footAngularWeight,
                          Vector3DReadOnly footLinearWeight)
   {
      swingState.setWeights(footAngularWeight, footLinearWeight);
      moveViaWaypointsState.setWeights(footAngularWeight, footLinearWeight);
      onToesState.setWeights(loadedFootAngularWeight, loadedFootLinearWeight);
      supportState.setWeights(loadedFootAngularWeight, loadedFootLinearWeight);
   }

   public void setAdjustedFootstepAndTime(Footstep adjustedFootstep, double swingTime)
   {
      setAdjustedFootstepAndTime(adjustedFootstep, null, null, swingTime);
   }

   public void setAdjustedFootstepAndTime(Footstep adjustedFootstep, FrameVector3DReadOnly finalCoMVelocity, FrameVector3DReadOnly finalCoMAcceleration, double swingTime)
   {
      swingState.setAdjustedFootstepAndTime(adjustedFootstep,finalCoMVelocity, finalCoMAcceleration, swingTime);
   }

   public void requestTouchdownForDisturbanceRecovery()
   {
      if (stateMachine.getCurrentState() == moveViaWaypointsState)
         moveViaWaypointsState.requestTouchdownForDisturbanceRecovery(stateMachine.getTimeInCurrentState());
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

   public void saveCurrentPositionAsLastFootstepPosition()
   {
      footControlHelper.getSwingTrajectoryCalculator().saveCurrentPositionAsLastFootstepPosition();
   }

   public void initializeSwingTrajectoryPreview(Footstep footstep, double swingDuration)
   {
      SwingTrajectoryCalculator swingTrajectoryCalculator = footControlHelper.getSwingTrajectoryCalculator();
      swingTrajectoryCalculator.setSwingDuration(swingDuration);
      swingTrajectoryCalculator.setInitialConditionsToCurrent();
      swingTrajectoryCalculator.setFootstep(footstep);
      swingTrajectoryCalculator.setShouldVisualize(false);
      swingTrajectoryCalculator.initializeTrajectoryWaypoints(true);
   }

   public void updateSwingTrajectoryPreview()
   {
      SwingTrajectoryCalculator swingTrajectoryCalculator = footControlHelper.getSwingTrajectoryCalculator();
      if (swingTrajectoryCalculator.getActiveTrajectoryType() != TrajectoryType.WAYPOINTS && swingTrajectoryCalculator.doOptimizationUpdate())
         swingTrajectoryCalculator.initializeTrajectoryWaypoints(false);
   }


   public void doControl()
   {
      controllerToolbox.setFootContactCoefficientOfFriction(robotSide, coefficientOfFriction.getValue());

      if (workspaceLimiterControlModule != null)
         workspaceLimiterControlModule.resetSwingParameters();

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

   public boolean isUsingPointContactInToeOff()
   {
      return onToesState.isUsingPointContact();
   }

   public void updateLegSingularityModule()
   {
      if (workspaceLimiterControlModule != null)
         workspaceLimiterControlModule.update();
   }

   public boolean correctCoMHeightTrajectoryForSupportSingularityAvoidance(CoMHeightTimeDerivativesDataBasics comHeightDataToCorrect,
                                                                           double zCurrent,
                                                                           ReferenceFrame pelvisZUpFrame)
   {
      if (workspaceLimiterControlModule != null)
      {
         return workspaceLimiterControlModule.correctCoMHeightTrajectoryForSingularityAvoidanceInSupport(comHeightDataToCorrect,
                                                                                                         zCurrent,
                                                                                                         pelvisZUpFrame,
                                                                                                         getCurrentConstraintType());
      }

      return false;
   }

   public boolean correctCoMHeightTrajectoryForUnreachableFootStep(CoMHeightTimeDerivativesDataBasics comHeightDataToCorrect)
   {
      if (workspaceLimiterControlModule != null)
      {
         return workspaceLimiterControlModule.correctCoMHeightTrajectoryForUnreachableFootStep(comHeightDataToCorrect, getCurrentConstraintType());
      }

      return false;
   }

   public void setFootstep(Footstep footstep, double swingTime)
   {
      setFootstep(footstep, swingTime, null, null);
   }

   public void setFootstep(Footstep footstep, double swingTime, FrameVector3DReadOnly finalCoMVelocity, FrameVector3DReadOnly finalCoMAcceleration)
   {
      swingState.setFootstep(footstep, swingTime, finalCoMVelocity, finalCoMAcceleration);
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command)
   {
      moveViaWaypointsState.handleFootTrajectoryCommand(command);
   }

   public void resetHeightCorrectionParametersForSingularityAvoidance()
   {
      if (workspaceLimiterControlModule != null)
         workspaceLimiterControlModule.resetHeightCorrectionParameters();
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

   /**
    * Computes and returns the swing time remaining while accounting for the active swing time speed up factor.
    * 
    * @return the estimated swing time remaining.
    */
   public double getSwingTimeRemaining()
   {
      if (stateMachine.getCurrentState() != swingState)
         return Double.NaN;
      else
         return swingState.getSwingTimeRemaining();
   }

   public double getFractionThroughSwing()
   {
      if (stateMachine.getCurrentState() != swingState)
         return Double.NaN;
      else
         return swingState.getFractionThroughSwing();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(stateMachine.getCurrentState().getInverseDynamicsCommand());
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
      if (!isInFlatSupportState())
         return;
      requestExploration.set(false);
      initializeFootExploration();
   }

   private void resetFootPolygon()
   {
      if (!isInFlatSupportState())
         return;
      resetFootPolygon.set(false);
      if (footControlHelper.getPartialFootholdControlModule() != null)
      {
         footControlHelper.getPartialFootholdControlModule().reset();
      }
      controllerToolbox.resetFootSupportPolygon(robotSide);
   }

   public void unload(double percentInUnloading, double rhoMin)
   {
      if (minWrenchCommand == null)
         return;
      minZForce.set((1.0 - percentInUnloading) * minWeightFractionPerFoot.getValue() * robotWeightFz);
      maxZForce.set((1.0 - percentInUnloading) * maxWeightFractionPerFoot.getValue() * robotWeightFz);

      // Make sure the max force is always a little larger then the min force required by the rhoMin value. This is to avoid sending conflicting constraints.
      maxZForce.set(Math.max(maxZForce.getValue(), computeMinZForceBasedOnRhoMin(rhoMin) + 1.0E-5));

      updateWrenchCommands();
   }

   private final FrameVector3D normalVector = new FrameVector3D();

   private double computeMinZForceBasedOnRhoMin(double rhoMin)
   {
      YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);

      contactState.getContactNormalFrameVector(normalVector);
      normalVector.changeFrame(ReferenceFrame.getWorldFrame());
      normalVector.normalize();

      double friction = contactState.getCoefficientOfFriction();
      int points = contactState.getNumberOfContactPointsInContact();

      return normalVector.getZ() * rhoMin * numberOfBasisVectors * points / Math.sqrt(1.0 + friction * friction);
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

   public Object pollStatusToReport()
   {
      return stateMachine.getCurrentState().pollStatusToReport();
   }

   public void liftOff(double pitch, double pitchVelocity, double duration)
   {
      // Should not do this in the toe off state.
      if (getCurrentConstraintType() != ConstraintType.FULL)
         return;

      supportState.liftOff(pitch, pitchVelocity, duration);
   }

   public void touchDown(double initialPitch, double initialPitchVelocity, double pitch, double duration)
   {
      supportState.touchDown(initialPitch, initialPitchVelocity, pitch, duration);
   }

   public MultipleWaypointsPoseTrajectoryGenerator getSwingTrajectory()
   {
      return footControlHelper.getSwingTrajectoryCalculator().getSwingTrajectory();
   }
}