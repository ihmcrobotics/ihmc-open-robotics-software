package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.Arrays;
import java.util.EnumMap;

public class JumpingFootControlModule
{
   private final YoRegistry registry;
   private final ContactablePlaneBody contactableFoot;

   public enum ConstraintType
   {
      FULL, SWING;

      public boolean isLoadBearing()
      {
         switch (this)
         {
            case FULL:
               return true;
            default:
               return false;
         }
      }
   }

   private static final double defaultCoefficientOfFriction = 0.8;
   private final DoubleParameter coefficientOfFriction;

   private final StateMachine<ConstraintType, JumpingFootControlState> stateMachine;
   private final YoEnum<ConstraintType> requestedState;
   private final EnumMap<ConstraintType, boolean[]> contactStatesMap = new EnumMap<ConstraintType, boolean[]>(ConstraintType.class);

   private final JumpingControllerToolbox controllerToolbox;
   private final RobotSide robotSide;

   private final JumpingSwingFootState swingState;
   private final JumpingSupportFootState supportState;

   private final JumpingFootControlHelper footControlHelper;

   private final YoBoolean resetFootPolygon;

   public JumpingFootControlModule(RobotSide robotSide,
                                   WalkingControllerParameters walkingControllerParameters,
                                   PIDSE3GainsReadOnly swingFootControlGains,
                                   JumpingControllerToolbox controllerToolbox,
                                   YoRegistry parentRegistry)
   {
      contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      controllerToolbox.setFootContactStateFullyConstrained(robotSide);

      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Foot";
      registry = new YoRegistry(sidePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      footControlHelper = new JumpingFootControlHelper(robotSide, walkingControllerParameters, controllerToolbox, registry);

      this.controllerToolbox = controllerToolbox;
      this.robotSide = robotSide;

      requestedState = new YoEnum<>(namePrefix + "RequestedState", "", registry, ConstraintType.class, true);
      requestedState.set(null);

      setupContactStatesMap();

      supportState = new JumpingSupportFootState(footControlHelper, registry);
      swingState = new JumpingSwingFootState(footControlHelper, controllerToolbox.getGravityZ(), swingFootControlGains, registry);

      stateMachine = setupStateMachine(namePrefix);

      resetFootPolygon = new YoBoolean(namePrefix + "ResetFootPolygon", registry);

      String targetRegistryName = JumpingFeetManager.class.getSimpleName();
      String parameterRegistryName = JumpingFootControlModule.class.getSimpleName() + "Parameters";
      coefficientOfFriction = ParameterProvider.getOrCreateParameter(targetRegistryName,
                                                                     parameterRegistryName,
                                                                     "CoefficientOfFriction",
                                                                     registry,
                                                                     defaultCoefficientOfFriction);



   }

   private void setupContactStatesMap()
   {
      boolean[] falses = new boolean[contactableFoot.getTotalNumberOfContactPoints()];
      Arrays.fill(falses, false);
      boolean[] trues = new boolean[contactableFoot.getTotalNumberOfContactPoints()];
      Arrays.fill(trues, true);

      contactStatesMap.put(ConstraintType.SWING, falses);
      contactStatesMap.put(ConstraintType.FULL, trues);
   }

   private StateMachine<ConstraintType, JumpingFootControlState> setupStateMachine(String namePrefix)
   {
      StateMachineFactory<ConstraintType, JumpingFootControlState> factory = new StateMachineFactory<>(ConstraintType.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(footControlHelper.getJumpingControllerToolbox().getYoTime());
      factory.addState(ConstraintType.FULL, supportState);
      factory.addState(ConstraintType.SWING, swingState);

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
      supportState.setWeights(loadedFootAngularWeight, loadedFootLinearWeight);
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
   }

   public void doControl()
   {
      controllerToolbox.setFootContactCoefficientOfFriction(robotSide, coefficientOfFriction.getValue());

      if (resetFootPolygon.getBooleanValue())
      {
         resetFootPolygon();
      }

      stateMachine.doTransitions();

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

   public void initializeSwingTrajectoryPreview(FramePose3DReadOnly footstepPoseRelativeToTouchdownCoM, double swingHeight, double swingDuration)
   {
      JumpingSwingTrajectoryCalculator swingTrajectoryCalculator = footControlHelper.getSwingTrajectoryCalculator();
      swingTrajectoryCalculator.setInitialConditionsToCurrent();
      swingTrajectoryCalculator.setFootstep(footstepPoseRelativeToTouchdownCoM, swingHeight, swingDuration);
      swingTrajectoryCalculator.setShouldVisualize(false);
      swingTrajectoryCalculator.initializeTrajectoryWaypoints(true);
   }

   public void updateSwingTrajectoryPreview()
   {
      JumpingSwingTrajectoryCalculator swingTrajectoryCalculator = footControlHelper.getSwingTrajectoryCalculator();
      if (swingTrajectoryCalculator.doOptimizationUpdate())
         swingTrajectoryCalculator.initializeTrajectoryWaypoints(false);
   }

   public void setAdjustedFootstep(FramePose3DReadOnly adjustedFootstep)
   {
      swingState.setAdjustedFootstep(adjustedFootstep);
   }

   public void setFootstep(FramePose3DReadOnly footstepPoseRelativeToTouchdownCoM, double swingHeight, double swingTime)
   {
      swingState.setFootstep(footstepPoseRelativeToTouchdownCoM, swingHeight, swingTime);
   }

   public MultipleWaypointsPoseTrajectoryGenerator getSwingTrajectory()
   {
      return footControlHelper.getSwingTrajectoryCalculator().getSwingTrajectory();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return stateMachine.getCurrentState().getInverseDynamicsCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (ConstraintType constraintType : ConstraintType.values())
      {
         JumpingFootControlState state = stateMachine.getState(constraintType);
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }

   private void resetFootPolygon()
   {
      if (!isInFlatSupportState())
         return;
      resetFootPolygon.set(false);
      controllerToolbox.resetFootSupportPolygon(robotSide);
   }
}