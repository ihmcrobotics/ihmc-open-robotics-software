package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModule;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HighLevelHumanoidControllerPlugin;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HighLevelHumanoidControllerPluginFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.parameterEstimation.InertialParameterManager;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ControllerCrashLocation;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelControllerStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicListDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class HumanoidHighLevelControllerManager implements RobotController, SCS2YoGraphicHolder
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final StateMachine<HighLevelControllerName, HighLevelControllerState> stateMachine;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoBoolean isListeningToHighLevelStateMessage = new YoBoolean("isListeningToHighLevelStateMessage", registry);
   private final YoEnum<HighLevelControllerName> requestedHighLevelControllerState;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;

   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;
   private final JointDesiredOutputListBasics lowLevelControllerOutput;
   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final HighLevelControllerFactoryHelper controllerFactoryHelper;

   private final EnumMap<HighLevelControllerName, HighLevelControllerState> highLevelControllerStates = new EnumMap<>(HighLevelControllerName.class);
   private final List<HighLevelHumanoidControllerPlugin> controllerPlugins = new ArrayList<>();

   private final HighLevelStateChangeStatusMessage highLevelStateChangeStatusMessage = new HighLevelStateChangeStatusMessage();

   private final ExecutionTimer highLevelControllerTimer = new ExecutionTimer("activeHighLevelControllerTimer", 1.0, registry);

   private final RobotDesiredConfigurationData robotDesiredConfigurationData = new RobotDesiredConfigurationData();
   private final IntegerParameter jointDesiredOutputBroadcastFrequency = new IntegerParameter("jointDesiredOutputBroadcastFrequency", registry, 10);

   private final InertialParameterManager inertialParameterManager;

   public HumanoidHighLevelControllerManager(CommandInputManager commandInputManager,
                                             StatusMessageOutputManager statusMessageOutputManager,
                                             HighLevelControllerName initialControllerState,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             WalkingControllerParameters walkingControllerParameters,
                                             PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                             YoEnum<HighLevelControllerName> requestedHighLevelControllerState,
                                             EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerStateFactories,
                                             ArrayList<ControllerStateTransitionFactory<HighLevelControllerName>> controllerTransitionFactories,
                                             List<HighLevelHumanoidControllerPluginFactory> pluginFactories,
                                             HighLevelControlManagerFactory managerFactory,
                                             WholeBodyControllerCoreFactory controllerCoreFactory,
                                             HighLevelHumanoidControllerToolbox controllerToolbox,
                                             CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                             ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                             JointDesiredOutputListBasics lowLevelControllerOutput)
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
      controllerFactoryHelper.setParameters(highLevelControllerParameters, walkingControllerParameters, pushRecoveryControllerParameters);
      controllerFactoryHelper.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      controllerFactoryHelper.setLowLevelControllerOutput(lowLevelControllerOutput);
      controllerFactoryHelper.setRequestedHighLevelControllerState(requestedHighLevelControllerState);
      controllerFactoryHelper.setForceSensorDataHolder(forceSensorDataHolder);

      stateMachine = setUpStateMachine(initialControllerState,
                                       controllerStateFactories,
                                       controllerTransitionFactories,
                                       managerFactory,
                                       controllerCoreFactory,
                                       controllerToolbox.getYoTime(),
                                       registry);
      isListeningToHighLevelStateMessage.set(true);
      for (HighLevelControllerState highLevelControllerState : highLevelControllerStates.values())
      {
         this.registry.addChild(highLevelControllerState.getYoRegistry());
      }

      OneDoFJointBasics[] controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class);
      yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);

      pluginFactories.forEach(this::addControllerPluginFactory);

      inertialParameterManager = managerFactory.getOrCreateInertialParameterManager();
   }

   /**
    * Creates and registers a new plugin given its factory.
    * <p>
    * IMPORTANT: This should not be called when the controller is running, should only be used for
    * setup. Ideally, the plugin can be registered before creating the controller via the
    * {@link HighLevelHumanoidControllerFactory}.
    * </p>
    * 
    * @param pluginFactory the factory used to create the new plugin to be registered.
    */
   public void addControllerPluginFactory(HighLevelHumanoidControllerPluginFactory pluginFactory)
   {
      addControllerPlugin(pluginFactory.buildPlugin(controllerFactoryHelper));
   }

   /**
    * Registers a new plugin to be run alongside with the active controller.
    * <p>
    * IMPORTANT: This should not be called when the controller is running, should only be used for
    * setup. Ideally, the plugin can be registered before creating the controller via the
    * {@link HighLevelHumanoidControllerFactory}.
    * </p>
    * 
    * @param plugin the plugin to be registered.
    */
   public void addControllerPlugin(HighLevelHumanoidControllerPlugin plugin)
   {
      addYoVariableRegistry(plugin.getRegistry());
      controllerPlugins.add(plugin);
   }

   public void addYoVariableRegistry(YoRegistry registryToAdd)
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

         double time = controllerToolbox.getYoTime().getValue();

         for (int i = 0; i < controllerPlugins.size(); i++)
         {
            controllerPlugins.get(i).update(time);
         }

         stateMachine.doActionAndTransition();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         statusMessageOutputManager.reportStatusMessage(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.CONTROLLER_RUN, e));
         controllerToolbox.reportControllerFailureToListeners(null);
      }

      highLevelControllerTimer.stopMeasurement();

      copyJointDesiredsToJoints();
      reportDesiredCenterOfPressureForEstimator();
      reportRobotDesiredConfigurationData();

      if (inertialParameterManager != null)
         inertialParameterManager.update();
   }

   @Override
   public YoRegistry getYoRegistry()
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
                                                                                             HighLevelControlManagerFactory managerFactory,
                                                                                             WholeBodyControllerCoreFactory controllerCoreFactory,
                                                                                             YoDouble yoTime,
                                                                                             YoRegistry registry)
   {
      controllerFactoryHelper.setControllerFactories(controllerStateFactories);
      controllerFactoryHelper.setHighLevelControlManagerFactory(managerFactory);
      controllerFactoryHelper.setWholeBodyControllerCoreFactory(controllerCoreFactory);

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
            controllerToolbox.reportControllerStateChangeToListeners(from, to);
         }
      });

      factory.getRegisteredStates().forEach(state -> factory.addStateChangedListener((from, to) -> state.setPreviousHighLevelControllerName(from)));

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
         OneDoFJointReadOnly controlledJoint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(jointIndex);
         JointDesiredOutputReadOnly lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoint);

         if (!lowLevelJointData.hasControlMode())
            throw new NullPointerException("Joint: " + controlledJoint.getName() + " has no control mode.");
      }

      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
      lowLevelControllerOutput.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);

      RootJointDesiredConfigurationDataReadOnly rootJointDesiredConfiguration = stateMachine.getCurrentState().getOutputForRootJoint();
      if (rootJointDesiredConfiguration != null)
      {
         this.rootJointDesiredConfiguration.set(rootJointDesiredConfiguration);
      }
   }

   private int jointDesiredOutputBroadcastCounter = 0;

   private void reportRobotDesiredConfigurationData()
   {
      if (++jointDesiredOutputBroadcastCounter < jointDesiredOutputBroadcastFrequency.getValue())
         return;
      jointDesiredOutputBroadcastCounter = 0;

      lowLevelControllerOutput.copyToMessage(robotDesiredConfigurationData);

      HighLevelControllerState currentState = stateMachine.getCurrentState();
      if (currentState == null || currentState.getOutputForRootJoint() == null)
         return;

      currentState.getOutputForRootJoint().copyToMessage(robotDesiredConfigurationData);

      robotDesiredConfigurationData.setWallTime(System.nanoTime());
      // TODO use or remove joint name hash
      statusMessageOutputManager.reportStatusMessage(robotDesiredConfigurationData);
   }

   public HighLevelControllerName getCurrentHighLevelControlState()
   {
      return stateMachine.getCurrentStateKey();
   }

   public void addHighLevelStateChangedListener(StateChangedListener<HighLevelControllerName> stateChangedListener)
   {
      stateMachine.addStateChangedListener(stateChangedListener);
   }

   private final YoGraphicListDefinition scs2AdditionalYoGraphics = new YoGraphicListDefinition();

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(scs2AdditionalYoGraphics);
      WholeBodyControllerCoreFactory wholeBodyControllerCoreFactory = controllerFactoryHelper.getWholeBodyControllerCoreFactory();
      WholeBodyControllerCore wholeBodyControllerCore = wholeBodyControllerCoreFactory.getWholeBodyControllerCore();
      if (wholeBodyControllerCore != null)
         group.addChild(wholeBodyControllerCore.getSCS2YoGraphics());
      LinearMomentumRateControlModule linearMomentumRateControlModule = wholeBodyControllerCoreFactory.getLinearMomentumRateControlModule();
      if (linearMomentumRateControlModule != null)
         group.addChild(linearMomentumRateControlModule.getSCS2YoGraphics());
      if (inertialParameterManager != null)
         group.addChild(inertialParameterManager.getSCS2YoGraphics());
      group.addChild(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getSCS2YoGraphics());
      group.addChild(controllerFactoryHelper.getManagerFactory().getSCS2YoGraphics());
      for (HighLevelControllerState controllerState : highLevelControllerStates.values())
      {
         group.addChild(controllerState.getSCS2YoGraphics());
      }
      for (HighLevelHumanoidControllerPlugin controllerPlugin : controllerPlugins)
      {
         group.addChild(controllerPlugin.getSCS2YoGraphics());
      }
      return group;
   }

   public void addYoGraphic(YoGraphicDefinition scs2AdditionalYoGraphic)
   {
      scs2AdditionalYoGraphics.addYoGraphic(scs2AdditionalYoGraphic);
   }
}
