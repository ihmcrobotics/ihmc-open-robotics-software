package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.EnumMap;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ControllerCrashLocation;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelControllerStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class HumanoidHighLevelControllerManager implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final StateMachine<HighLevelControllerName, HighLevelControllerState> stateMachine;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoBoolean isListeningToHighLevelStateMessage = new YoBoolean("isListeningToHighLevelStateMessage", registry);
   private final YoEnum<HighLevelControllerName> requestedHighLevelControllerState;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;

   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;
   private final JointDesiredOutputList lowLevelControllerOutput;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final HighLevelControllerFactoryHelper controllerFactoryHelper;

   private final EnumMap<HighLevelControllerName, HighLevelControllerState> highLevelControllerStates = new EnumMap<>(HighLevelControllerName.class);

   private final HighLevelStateChangeStatusMessage highLevelStateChangeStatusMessage = new HighLevelStateChangeStatusMessage();

   private final ExecutionTimer highLevelControllerTimer = new ExecutionTimer("activeHighLevelControllerTimer", 1.0, registry);

   public HumanoidHighLevelControllerManager(CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager,
                                             HighLevelControllerName initialControllerState, HighLevelControllerParameters highLevelControllerParameters,
                                             WalkingControllerParameters walkingControllerParameters, ICPTrajectoryPlannerParameters icpPlannerParameters,
                                             YoEnum<HighLevelControllerName> requestedHighLevelControllerState,
                                             EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerStateFactories,
                                             ArrayList<ControllerStateTransitionFactory<HighLevelControllerName>> controllerTransitionFactories,
                                             HighLevelControlManagerFactory managerFactory, HighLevelHumanoidControllerToolbox controllerToolbox,
                                             CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                             ForceSensorDataHolderReadOnly forceSensorDataHolder, JointDesiredOutputList lowLevelControllerOutput)
   {
      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;
      this.controllerToolbox = controllerToolbox;
      this.requestedHighLevelControllerState = requestedHighLevelControllerState;
      this.centerOfPressureDataHolderForEstimator = centerOfPressureDataHolderForEstimator;
      this.lowLevelControllerOutput = lowLevelControllerOutput;

      this.requestedHighLevelControllerState.set(initialControllerState);
      registry.addChild(controllerToolbox.getYoVariableRegistry());

      controllerFactoryHelper = new HighLevelControllerFactoryHelper();
      controllerFactoryHelper.setCommandInputManager(commandInputManager);
      controllerFactoryHelper.setStatusMessageOutputManager(statusMessageOutputManager);
      controllerFactoryHelper.setParameters(highLevelControllerParameters, walkingControllerParameters, icpPlannerParameters);
      controllerFactoryHelper.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      controllerFactoryHelper.setLowLevelControllerOutput(lowLevelControllerOutput);
      controllerFactoryHelper.setRequestedHighLevelControllerState(requestedHighLevelControllerState);
      controllerFactoryHelper.setForceSensorDataHolder(forceSensorDataHolder);

      stateMachine = setUpStateMachine(initialControllerState, controllerStateFactories, controllerTransitionFactories, managerFactory,
                                       controllerToolbox.getYoTime(), registry);
      isListeningToHighLevelStateMessage.set(true);
      for (HighLevelControllerState highLevelControllerState : highLevelControllerStates.values())
      {
         this.registry.addChild(highLevelControllerState.getYoVariableRegistry());
      }

      OneDoFJointBasics[] controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class);
      yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);
   }

   public void addYoVariableRegistry(YoVariableRegistry registryToAdd)
   {
      this.registry.addChild(registryToAdd);
   }

   public void requestHighLevelControllerState(HighLevelControllerName requestedHighLevelControllerName)
   {
      this.requestedHighLevelControllerState.set(requestedHighLevelControllerName);
   }

   public void setListenToHighLevelStatePackets(boolean isListening)
   {
      isListeningToHighLevelStateMessage.set(isListening);
   }

   @Override
   public void initialize()
   {
      controllerToolbox.initialize();
      stateMachine.resetToInitialState();
   }

   @Override
   public void doControl()
   {
      if (isListeningToHighLevelStateMessage.getBooleanValue())
      {
         if (commandInputManager.isNewCommandAvailable(HighLevelControllerStateCommand.class))
         {
            requestedHighLevelControllerState.set(commandInputManager.pollNewestCommand(HighLevelControllerStateCommand.class).getHighLevelControllerName());
         }
      }

      highLevelControllerTimer.startMeasurement();

      try
      {
         controllerToolbox.update();
         stateMachine.doActionAndTransition();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         statusMessageOutputManager.reportStatusMessage(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.CONTROLLER_RUN,
                                                                                                             e.getMessage()));
         controllerToolbox.reportControllerFailureToListeners(null);
      }

      highLevelControllerTimer.stopMeasurement();

      copyJointDesiredsToJoints();
      reportDesiredCenterOfPressureForEstimator();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return this.getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   private StateMachine<HighLevelControllerName, HighLevelControllerState> setUpStateMachine(HighLevelControllerName initialControllerState,
                                                                                             EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerStateFactories,
                                                                                             ArrayList<ControllerStateTransitionFactory<HighLevelControllerName>> controllerTransitionFactories,
                                                                                             HighLevelControlManagerFactory managerFactory, YoDouble yoTime,
                                                                                             YoVariableRegistry registry)
   {
      controllerFactoryHelper.setControllerFactories(controllerStateFactories);
      controllerFactoryHelper.setHighLevelControlManagerFactory(managerFactory);

      StateMachineFactory<HighLevelControllerName, HighLevelControllerState> factory = new StateMachineFactory<>(HighLevelControllerName.class);
      factory.setNamePrefix("highLevelControllerName").setRegistry(registry).buildYoClock(yoTime);

      // create controller states
      for (HighLevelControllerStateFactory controllerStateFactory : controllerStateFactories.values())
      {
         // create the individual state
         HighLevelControllerState highLevelControllerState = controllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);

         // add the controller to the state machine
         factory.addState(highLevelControllerState.getHighLevelControllerName(), highLevelControllerState);

         highLevelControllerStates.put(controllerStateFactory.getStateEnum(), highLevelControllerState);

         if (controllerStateFactory.isTransitionToControllerRequested())
            requestedHighLevelControllerState.set(controllerStateFactory.getStateEnum());
      }

      // create controller transitions
      for (ControllerStateTransitionFactory<HighLevelControllerName> controllerStateTransitionFactory : controllerTransitionFactories)
      {
         StateTransition<HighLevelControllerName> stateTransition = controllerStateTransitionFactory.getOrCreateStateTransition(highLevelControllerStates,
                                                                                                                                controllerFactoryHelper,
                                                                                                                                registry);
         HighLevelControllerName stateToAttachEnum = controllerStateTransitionFactory.getStateToAttachEnum();
         factory.addTransition(stateToAttachEnum, stateTransition);
      }

      factory.addStateChangedListener(new StateChangedListener<HighLevelControllerName>()
      {
         @Override
         public void stateChanged(HighLevelControllerName from, HighLevelControllerName to)
         {
            byte fromByte = from == null ? -1 : from.toByte();
            byte toByte = to == null ? -1 : to.toByte();
            highLevelStateChangeStatusMessage.setInitialHighLevelControllerName(fromByte);
            highLevelStateChangeStatusMessage.setEndHighLevelControllerName(toByte);
            statusMessageOutputManager.reportStatusMessage(highLevelStateChangeStatusMessage);
         }
      });

      return factory.build(initialControllerState);
   }

   private final SideDependentList<FramePoint2D> desiredFootCoPs = new SideDependentList<FramePoint2D>(new FramePoint2D(), new FramePoint2D());

   private void reportDesiredCenterOfPressureForEstimator()
   {
      SideDependentList<? extends ContactablePlaneBody> contactableFeet = controllerToolbox.getContactableFeet();
      FullHumanoidRobotModel fullHumanoidRobotModel = controllerToolbox.getFullRobotModel();
      for (RobotSide robotSide : RobotSide.values)
      {
         controllerToolbox.getDesiredCenterOfPressure(contactableFeet.get(robotSide), desiredFootCoPs.get(robotSide));
         centerOfPressureDataHolderForEstimator.setCenterOfPressure(desiredFootCoPs.get(robotSide), fullHumanoidRobotModel.getFoot(robotSide));
      }
   }

   private void copyJointDesiredsToJoints()
   {
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = stateMachine.getCurrentState().getOutputForLowLevelController();
      for (int jointIndex = 0; jointIndex < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithDesiredOutput(); jointIndex++)
      {
         OneDoFJointBasics controlledJoint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(jointIndex);
         JointDesiredOutputReadOnly lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoint);

         if (!lowLevelJointData.hasControlMode())
            throw new NullPointerException("Joint: " + controlledJoint.getName() + " has no control mode.");
      }

      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
      lowLevelControllerOutput.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
   }

   public HighLevelControllerName getCurrentHighLevelControlState()
   {
      return stateMachine.getCurrentStateKey();
   }
}
