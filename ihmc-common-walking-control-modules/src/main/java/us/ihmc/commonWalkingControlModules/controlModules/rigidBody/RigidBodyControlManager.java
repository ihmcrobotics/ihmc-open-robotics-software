package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.DesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.LoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WrenchTrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class RigidBodyControlManager
{
   public static final double INITIAL_GO_HOME_TIME = 2.0;

   private final String bodyName;
   private final YoVariableRegistry registry;
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

   public RigidBodyControlManager(RigidBodyBasics bodyToControl, RigidBodyBasics baseBody, RigidBodyBasics elevator,
                                  TObjectDoubleHashMap<String> homeConfiguration, Pose3D homePose, ReferenceFrame controlFrame, ReferenceFrame baseFrame,
                                  Vector3DReadOnly taskspaceAngularWeight, Vector3DReadOnly taskspaceLinearWeight, PID3DGainsReadOnly taskspaceOrientationGains,
                                  PID3DGainsReadOnly taskspacePositionGains, ContactablePlaneBody contactableBody, RigidBodyControlMode defaultControlMode,
                                  YoDouble yoTime, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      bodyName = bodyToControl.getName();
      String namePrefix = bodyName + "Manager";
      registry = new YoVariableRegistry(namePrefix);

      requestedState = new YoEnum<>(namePrefix + "RequestedControlMode", registry, RigidBodyControlMode.class, true);
      stateSwitched = new YoBoolean(namePrefix + "StateSwitched", registry);

      jointsToControl = MultiBodySystemTools.createOneDoFJointPath(baseBody, bodyToControl);

      initialJointPositions = new double[jointsToControl.length];

      RigidBodyJointControlHelper jointControlHelper = new RigidBodyJointControlHelper(bodyName, jointsToControl, parentRegistry);

      jointspaceControlState = new RigidBodyJointspaceControlState(bodyName, jointsToControl, homeConfiguration, yoTime, jointControlHelper, registry);

      if (taskspaceAngularWeight != null && taskspaceLinearWeight == null)
      {
         RigidBodyOrientationController taskspaceControlState = new RigidBodyOrientationController(bodyToControl, baseBody, elevator, baseFrame, yoTime,
                                                                                                   jointControlHelper, parentRegistry);
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
         RigidBodyPositionController taskspaceControlState = new RigidBodyPositionController(bodyToControl, baseBody, elevator, controlFrame, baseFrame, yoTime,
                                                                                             parentRegistry, graphicsListRegistry);
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
         RigidBodyPoseController taskspaceControlState = new RigidBodyPoseController(bodyToControl, baseBody, elevator, controlFrame, baseFrame, yoTime,
                                                                                     jointControlHelper, graphicsListRegistry, registry);
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
         loadBearingControlState = new RigidBodyLoadBearingControlState(bodyToControl, contactableBody, elevator, yoTime, jointControlHelper,
                                                                        graphicsListRegistry, registry);
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
      this.defaultControlMode = new EnumParameter<>(namePrefix + "DefaultControlMode", description, registry, RigidBodyControlMode.class, false,
                                                    defaultControlMode);
      this.defaultControlMode.addParameterChangedListener(parameter -> checkDefaultControlMode(this.defaultControlMode.getValue(), this.homePose, bodyName));

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

   public void setGains(Map<String, PIDGainsReadOnly> jointspaceGains)
   {
      jointspaceControlState.setGains(jointspaceGains);
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
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid orientation trajectory command.");
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
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid pose trajectory command.");
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
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid jointspace trajectory command.");
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
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid hybrid SE3 trajectory command.");
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
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid hybrid SO3 trajectory command.");
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
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid desired accelerations command.");
         hold();
      }
   }

   public void handleWrenchTrajectoryCommand(WrenchTrajectoryControllerCommand command)
   {
      if (!externalWrenchManager.handleWrenchTrajectoryCommand(command))
      {
         LogTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid wrench trajectory command.");
         externalWrenchManager.clear();
      }
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
}
