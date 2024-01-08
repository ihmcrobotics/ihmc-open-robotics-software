package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.Map;

/**
 * Manages a rigid body as part of a high level controller.
 * <p>
 * This class triages user commands and computes inverse dynamics, feedback control commands,
 * and joint desired output data that are provided to the whole body controller core.
 * </p>
 * <p>
 * As part of this class, a rigid body can be in one of the four {@link RigidBodyControlMode
 * rigid body control modes}.
 * </p>
 */
public class RigidBodyControlManager implements SCS2YoGraphicHolder
{
   public static final double INITIAL_GO_HOME_TIME = 2.0;

   private final String bodyName;
   private final YoRegistry registry;
   private final StateMachine<RigidBodyControlMode, RigidBodyControlState> stateMachine;
   private final YoEnum<RigidBodyControlMode> requestedState;
   private final EnumParameter<RigidBodyControlMode> defaultControlMode;

   private final RigidBodyJointspaceControlState jointspaceControlState;
   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private final RigidBodyUserControlState userControlState;
   private final RigidBodyLoadBearingControlState loadBearingControlState;
   private final RigidBodyExternalWrenchManager externalWrenchManager;

   private final double[] initialJointPositions;
   private final FramePose3D homePose;

   private final OneDoFJointBasics[] jointsToControl;

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final YoBoolean stateSwitched;
   private final YoBoolean doPrepareForLocomotion;

   public RigidBodyControlManager(RigidBodyBasics bodyToControl,
                                  RigidBodyBasics baseBody,
                                  RigidBodyBasics elevator,
                                  TObjectDoubleHashMap<String> homeConfiguration,
                                  Pose3D homePose,
                                  ReferenceFrame controlFrame,
                                  ReferenceFrame baseFrame,
                                  Vector3DReadOnly taskspaceAngularWeight,
                                  Vector3DReadOnly taskspaceLinearWeight,
                                  PID3DGainsReadOnly taskspaceOrientationGains,
                                  PID3DGainsReadOnly taskspacePositionGains,
                                  ContactablePlaneBody contactableBody,
                                  RigidBodyControlMode defaultControlMode,
                                  boolean enableFunctionGenerators,
                                  YoDouble yoTime,
                                  YoGraphicsListRegistry graphicsListRegistry,
                                  YoRegistry parentRegistry)
   {
      bodyName = bodyToControl.getName();
      String namePrefix = bodyName + "Manager";
      registry = new YoRegistry(namePrefix);

      requestedState = new YoEnum<>(namePrefix + "RequestedControlMode", registry, RigidBodyControlMode.class, true);
      stateSwitched = new YoBoolean(namePrefix + "StateSwitched", registry);

      doPrepareForLocomotion = new YoBoolean(namePrefix + "DoPrepareForLocomotion", registry);

      jointsToControl = MultiBodySystemTools.createOneDoFJointPath(baseBody, bodyToControl);

      initialJointPositions = new double[jointsToControl.length];

      RigidBodyJointControlHelper jointControlHelper = new RigidBodyJointControlHelper(bodyName, jointsToControl, yoTime, enableFunctionGenerators, parentRegistry);

      jointspaceControlState = new RigidBodyJointspaceControlState(bodyName, jointsToControl, homeConfiguration, yoTime, jointControlHelper, registry);

      if (taskspaceAngularWeight != null && taskspaceLinearWeight == null)
      {
         RigidBodyOrientationController taskspaceControlState = new RigidBodyOrientationController(bodyToControl,
                                                                                                   baseBody,
                                                                                                   elevator,
                                                                                                   baseFrame,
                                                                                                   yoTime,
                                                                                                   jointControlHelper,
                                                                                                   enableFunctionGenerators,
                                                                                                   parentRegistry);
         if (taskspaceOrientationGains == null)
         {
            throw new RuntimeException("Can not create orientation control manager with null gains for " + bodyName);
         }
         taskspaceControlState.setGains(taskspaceOrientationGains);
         taskspaceControlState.setWeights(taskspaceAngularWeight);
         this.taskspaceControlState = taskspaceControlState;
         LogTools.info("Creating manager for " + bodyName + " with orientation controller.");
      }
      else if (taskspaceAngularWeight == null && taskspaceLinearWeight != null)
      {
         RigidBodyPositionController taskspaceControlState = new RigidBodyPositionController(bodyToControl,
                                                                                             baseBody,
                                                                                             elevator,
                                                                                             controlFrame,
                                                                                             baseFrame,
                                                                                             yoTime,
                                                                                             enableFunctionGenerators,
                                                                                             parentRegistry,
                                                                                             graphicsListRegistry);
         if (taskspacePositionGains == null)
         {
            throw new RuntimeException("Can not create position control manager with null gains for " + bodyName);
         }
         taskspaceControlState.setGains(taskspacePositionGains);
         taskspaceControlState.setWeights(taskspaceLinearWeight);
         this.taskspaceControlState = taskspaceControlState;
         LogTools.info("Creating manager for " + bodyName + " with position controller.");
      }
      else if (taskspaceAngularWeight != null && taskspaceLinearWeight != null)
      {
         RigidBodyPoseController taskspaceControlState = new RigidBodyPoseController(bodyToControl,
                                                                                     baseBody,
                                                                                     elevator,
                                                                                     controlFrame,
                                                                                     baseFrame,
                                                                                     yoTime,
                                                                                     jointControlHelper,
                                                                                     enableFunctionGenerators,
                                                                                     graphicsListRegistry,
                                                                                     registry);
         if (taskspaceOrientationGains == null || taskspacePositionGains == null)
         {
            System.out.println("Orientation gains exist: " + (taskspaceOrientationGains != null));
            System.out.println("Position gains exist: " + (taskspacePositionGains != null));
            throw new RuntimeException("Can not create pose control manager with null gains for " + bodyName);
         }
         taskspaceControlState.setGains(taskspaceOrientationGains, taskspacePositionGains);
         taskspaceControlState.setWeights(taskspaceAngularWeight, taskspaceLinearWeight);
         this.taskspaceControlState = taskspaceControlState;
         LogTools.info("Creating manager for " + bodyName + " with pose controller.");
      }
      else
      {
         throw new RuntimeException("No gains or weights for " + bodyName);
      }

      userControlState = new RigidBodyUserControlState(bodyName, jointsToControl, yoTime, registry);

      if (contactableBody != null)
      {
         loadBearingControlState = new RigidBodyLoadBearingControlState(bodyToControl,
                                                                        contactableBody,
                                                                        elevator,
                                                                        yoTime,
                                                                        jointControlHelper,
                                                                        graphicsListRegistry,
                                                                        registry);
         loadBearingControlState.setGains(taskspaceOrientationGains, taskspacePositionGains);
         loadBearingControlState.setWeights(taskspaceAngularWeight, taskspaceLinearWeight);
      }
      else
      {
         loadBearingControlState = null;
      }

      if (homePose != null)
         this.homePose = new FramePose3D(baseFrame, homePose);
      else
         this.homePose = null;

      externalWrenchManager = new RigidBodyExternalWrenchManager(bodyToControl, baseBody, controlFrame, yoTime, graphicsListRegistry, registry);

      defaultControlMode = defaultControlMode == null ? RigidBodyControlMode.JOINTSPACE : defaultControlMode;
      checkDefaultControlMode(defaultControlMode, this.homePose, bodyName);
      String description = "WARNING: only " + RigidBodyControlMode.JOINTSPACE + " or " + RigidBodyControlMode.TASKSPACE + " possible!";
      this.defaultControlMode = new EnumParameter<>(namePrefix
            + "DefaultControlMode", description, registry, RigidBodyControlMode.class, false, defaultControlMode);
      this.defaultControlMode.addListener(parameter -> checkDefaultControlMode(this.defaultControlMode.getValue(), this.homePose, bodyName));

      stateMachine = setupStateMachine(namePrefix, yoTime);
      parentRegistry.addChild(registry);
   }

   private StateMachine<RigidBodyControlMode, RigidBodyControlState> setupStateMachine(String namePrefix, DoubleProvider timeProvider)
   {
      StateMachineFactory<RigidBodyControlMode, RigidBodyControlState> factory = new StateMachineFactory<>(RigidBodyControlMode.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(RigidBodyControlMode.JOINTSPACE, jointspaceControlState);
      factory.addState(RigidBodyControlMode.TASKSPACE, taskspaceControlState);
      factory.addState(RigidBodyControlMode.USER, userControlState);
      if (loadBearingControlState != null)
         factory.addState(RigidBodyControlMode.LOADBEARING, loadBearingControlState);

      for (RigidBodyControlMode from : factory.getRegisteredStateKeys())
      {
         for (RigidBodyControlMode to : factory.getRegisteredStateKeys())
         {
            factory.addRequestedTransition(from, to, requestedState);
         }
      }

      return factory.build(RigidBodyControlMode.JOINTSPACE);
   }

   public void setWeights(Map<String, DoubleProvider> jointspaceWeights, Map<String, DoubleProvider> userModeWeights)
   {
      jointspaceControlState.setDefaultWeights(jointspaceWeights);
      userControlState.setWeights(userModeWeights);
   }

   public void setGains(Map<String, PIDGainsReadOnly> jointspaceHighLevelGains, Map<String, PIDGainsReadOnly> jointspaceLowLevelGains)
   {
      jointspaceControlState.setGains(jointspaceHighLevelGains, jointspaceLowLevelGains);
   }

   /**
    * Sets the behavior for {@link #prepareForLocomotion()}.
    * 
    * @param value whether {@link #prepareForLocomotion()} should be enabled or not.
    */
   public void setDoPrepareForLocomotion(boolean value)
   {
      doPrepareForLocomotion.set(value);
   }

   private static void checkDefaultControlMode(RigidBodyControlMode defaultControlMode, FramePose3D homePose, String bodyName)
   {
      if (defaultControlMode == null)
      {
         throw new RuntimeException("Default control mode can not be null for body " + bodyName + ".");
      }

      if (defaultControlMode == RigidBodyControlMode.TASKSPACE && homePose == null)
      {
         throw new RuntimeException("Need to define home pose if default control mode for body " + bodyName + " is set to TASKSPACE.");
      }

      if (defaultControlMode != RigidBodyControlMode.TASKSPACE && defaultControlMode != RigidBodyControlMode.JOINTSPACE)
      {
         throw new RuntimeException("Only JOINTSPACE or TASKSPACE control modes are allowed as default modes for body " + bodyName + ".");
      }
   }

   public void initialize()
   {
      stateMachine.resetToInitialState();
      goToHomeFromCurrent(INITIAL_GO_HOME_TIME);
   }

   public void compute()
   {
      if (stateMachine.getCurrentState().abortState())
         hold();

      stateSwitched.set(stateMachine.doTransitions());
      stateMachine.doAction();

      externalWrenchManager.doAction(Double.NaN);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      if (command.isStopAllTrajectory())
      {
         holdCurrentDesired();
         externalWrenchManager.clear();
      }
   }

   public void handleTaskspaceTrajectoryCommand(SO3TrajectoryControllerCommand command)
   {
      if (taskspaceControlState.handleTrajectoryCommand(command))
      {
         requestState(taskspaceControlState.getControlMode());
      }
      else
      {
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " received invalid orientation trajectory command.");
         hold();
      }
   }

   public void handleTaskspaceTrajectoryCommand(SE3TrajectoryControllerCommand command)
   {
      if (taskspaceControlState.handleTrajectoryCommand(command))
      {
         requestState(taskspaceControlState.getControlMode());
      }
      else
      {
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " received invalid pose trajectory command.");
         hold();
      }
   }

   public void handleJointspaceTrajectoryCommand(JointspaceTrajectoryCommand command)
   {
      computeDesiredJointPositions(initialJointPositions);

      if (jointspaceControlState.handleTrajectoryCommand(command, initialJointPositions))
      {
         requestState(jointspaceControlState.getControlMode());
      }
      else
      {
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " received invalid jointspace trajectory command.");
         hold();
      }
   }

   public void handleHybridTrajectoryCommand(SE3TrajectoryControllerCommand taskspaceCommand, JointspaceTrajectoryCommand jointSpaceCommand)
   {
      computeDesiredJointPositions(initialJointPositions);

      if (taskspaceControlState.handleHybridTrajectoryCommand(taskspaceCommand, jointSpaceCommand, initialJointPositions))
      {
         requestState(taskspaceControlState.getControlMode());
      }
      else
      {
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " received invalid hybrid SE3 trajectory command.");
         hold();
      }
   }

   public void handleHybridTrajectoryCommand(SO3TrajectoryControllerCommand taskspaceCommand, JointspaceTrajectoryCommand jointSpaceCommand)
   {
      computeDesiredJointPositions(initialJointPositions);

      if (taskspaceControlState.handleHybridTrajectoryCommand(taskspaceCommand, jointSpaceCommand, initialJointPositions))
      {
         requestState(taskspaceControlState.getControlMode());
      }
      else
      {
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " received invalid hybrid SO3 trajectory command.");
         hold();
      }
   }

   public void handleDesiredAccelerationsCommand(DesiredAccelerationsCommand command)
   {
      if (userControlState.handleDesiredAccelerationsCommand(command))
      {
         requestState(userControlState.getControlMode());
      }
      else
      {
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " received invalid desired accelerations command.");
         hold();
      }
   }

   public void handleWrenchTrajectoryCommand(WrenchTrajectoryControllerCommand command)
   {
      if (!externalWrenchManager.handleWrenchTrajectoryCommand(command))
      {
         externalWrenchManager.clear();
      }
   }

   /**
    * When enabled, i.e. {@code doPrepareForLocomotion == true}, it is equivalent to calling
    * {@link #holdInJointspace()}.
    */
   public void prepareForLocomotion()
   {
      if (doPrepareForLocomotion.getValue())
         holdCurrentDesired();
   }

   public void holdInJointspace()
   {
      jointspaceControlState.holdCurrent();
      requestState(jointspaceControlState.getControlMode());
   }

   public void holdCurrentDesiredInJointspace()
   {
      // It is only safe to hold the current desired if the body was controlled in the control mode. Otherwise the
      // desired values might be out of date or they might have never been set. In that case hold the current.
      if (getActiveControlMode() == jointspaceControlState.getControlMode())
      {
         jointspaceControlState.holdCurrentDesired();
         requestState(jointspaceControlState.getControlMode());
      }
      else
      {
         holdInJointspace();
      }
   }

   public void holdInTaskspace()
   {
      taskspaceControlState.holdCurrent();
      requestState(taskspaceControlState.getControlMode());
   }

   public void holdCurrentDesiredInTaskspace()
   {
      // It is only safe to hold the current desired if the body was controlled in the control mode. Otherwise the
      // desired values might be out of date or they might have never been set. In that case hold the current.
      if (getActiveControlMode() == taskspaceControlState.getControlMode())
      {
         taskspaceControlState.holdCurrentDesired();
         requestState(taskspaceControlState.getControlMode());
      }
      else
      {
         holdInTaskspace();
      }
   }

   public void hold()
   {
      switch (defaultControlMode.getValue())
      {
         case JOINTSPACE:
            holdInJointspace();
            break;
         case TASKSPACE:
            holdInTaskspace();
            break;
         default:
            throw new RuntimeException("Default control mode " + defaultControlMode.getValue() + " is not an implemented option.");
      }
   }

   public void holdCurrentDesired()
   {
      switch (defaultControlMode.getValue())
      {
         case JOINTSPACE:
            holdCurrentDesiredInJointspace();
            break;
         case TASKSPACE:
            holdCurrentDesiredInTaskspace();
            break;
         default:
            throw new RuntimeException("Default control mode " + defaultControlMode.getValue() + " is not an implemented option.");
      }
   }

   public void goToHomeFromCurrent(double trajectoryTime)
   {
      switch (defaultControlMode.getValue())
      {
         case JOINTSPACE:
            jointspaceControlState.goHomeFromCurrent(trajectoryTime);
            requestState(jointspaceControlState.getControlMode());
            break;
         case TASKSPACE:
            taskspaceControlState.goToPoseFromCurrent(homePose, trajectoryTime);
            requestState(taskspaceControlState.getControlMode());
            break;
         default:
            throw new RuntimeException("Default control mode " + defaultControlMode.getValue() + " is not an implemented option.");
      }
   }

   public void goHome(double trajectoryTime)
   {
      switch (defaultControlMode.getValue())
      {
         case JOINTSPACE:
            computeDesiredJointPositions(initialJointPositions);
            jointspaceControlState.goHome(trajectoryTime, initialJointPositions);
            requestState(jointspaceControlState.getControlMode());
            break;
         case TASKSPACE:
            taskspaceControlState.goToPose(homePose, trajectoryTime);
            requestState(taskspaceControlState.getControlMode());
            break;
         default:
            throw new RuntimeException("Default control mode " + defaultControlMode.getValue() + " is not an implemented option.");
      }
   }

   public void handleLoadBearingCommand(LoadBearingCommand command, JointspaceTrajectoryCommand jointspaceCommand)
   {
      if (loadBearingControlState == null)
      {
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " can not go to load bearing.");
         return;
      }

      if (!command.getLoad())
      {
         hold();
         return;
      }

      if (jointspaceCommand != null)
      {
         computeDesiredJointPositions(initialJointPositions);
         if (!loadBearingControlState.handleJointTrajectoryCommand(jointspaceCommand, initialJointPositions))
            return;
      }

      if (loadBearingControlState.handleLoadbearingCommand(command))
      {
         requestState(loadBearingControlState.getControlMode());
      }
   }

   public boolean isLoadBearing()
   {
      if (loadBearingControlState == null)
         return false;

      return stateMachine.getCurrentStateKey() == loadBearingControlState.getControlMode();
   }

   public void resetJointIntegrators()
   {
      // FIXME
      //      for (int jointIdx = 0; jointIdx < jointsToControl.length; jointIdx++)
      //         jointsToControl[jointIdx].resetIntegrator();
   }

   private void computeDesiredJointPositions(double[] desiredJointPositionsToPack)
   {
      if (stateMachine.getCurrentStateKey() == jointspaceControlState.getControlMode())
      {
         for (int i = 0; i < jointsToControl.length; i++)
            desiredJointPositionsToPack[i] = jointspaceControlState.getJointDesiredPosition(i);
      }
      else
      {
         for (int i = 0; i < jointsToControl.length; i++)
            desiredJointPositionsToPack[i] = jointsToControl[i].getQ();
      }
   }

   private void requestState(RigidBodyControlMode state)
   {
      if (stateMachine.getCurrentStateKey() != state)
         requestedState.set(state);
   }

   public RigidBodyControlMode getActiveControlMode()
   {
      return stateMachine.getCurrentStateKey();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(stateMachine.getCurrentState().getInverseDynamicsCommand());
      inverseDynamicsCommandList.addCommand(externalWrenchManager.getInverseDynamicsCommand());

      if (stateSwitched.getBooleanValue())
      {
         RigidBodyControlState previousState = stateMachine.getPreviousState();
         InverseDynamicsCommand<?> transitionOutOfStateCommand = previousState.getTransitionOutOfStateCommand();
         inverseDynamicsCommandList.addCommand(transitionOutOfStateCommand);
      }

      return inverseDynamicsCommandList;
   }

   public JointDesiredOutputListReadOnly getJointDesiredData()
   {
      return stateMachine.getCurrentState().getJointDesiredData();
   }

   public void setEnableDirectJointPositionControl(boolean enable)
   {
      jointspaceControlState.setEnableDirectJointPositionControl(enable);
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (RigidBodyControlMode mode : RigidBodyControlMode.values())
      {
         RigidBodyControlState state = stateMachine.getState(mode);
         if (state != null && state.createFeedbackControlTemplate() != null)
            ret.addCommand(state.createFeedbackControlTemplate());
      }
      return ret;
   }

   public OneDoFJointBasics[] getControlledJoints()
   {
      return jointsToControl;
   }

   public Object pollStatusToReport()
   {
      return stateMachine.getCurrentState().pollStatusToReport();
   }

   public RigidBodyTaskspaceControlState getTaskspaceControlState()
   {
      return taskspaceControlState;
   }

   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      if (loadBearingControlState != null)
         loadBearingControlState.setControllerCoreOutput(controllerCoreOutput);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(bodyName + "-" + getClass().getSimpleName());
      group.addChild(taskspaceControlState.getSCS2YoGraphics());
      if (loadBearingControlState != null)
         group.addChild(loadBearingControlState.getSCS2YoGraphics());
      group.addChild(externalWrenchManager.getSCS2YoGraphics());
      return group.isEmpty() ? null : group;
   }
}
