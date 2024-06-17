package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.DefaultSplitFractionCalculatorParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionCalculatorParametersReadOnly;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired.UserDesiredControllerCommandGenerators;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.QueuedControllerCommandGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.falling.FallingControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HumanoidHighLevelControllerManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ComponentBasedFootstepDataMessageGeneratorFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HighLevelHumanoidControllerPluginFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.ClearDelayQueueConverter;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameMessageCommandConverter;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfMassStateProvider;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class HighLevelHumanoidControllerFactory implements CloseableAndDisposable
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   private final ArrayList<HighLevelControllerStateFactory> controllerStateFactories = new ArrayList<>();
   private final EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerFactoriesMap = new EnumMap<>(HighLevelControllerName.class);

   private final ArrayList<ControllerStateTransitionFactory<HighLevelControllerName>> stateTransitionFactories = new ArrayList<>();

   private HighLevelHumanoidControllerToolbox controllerToolbox = null;

   private final YoEnum<HighLevelControllerName> requestedHighLevelControllerState = new YoEnum<>("requestedHighLevelControllerState",
                                                                                                  registry,
                                                                                                  HighLevelControllerName.class,
                                                                                                  true);
   private HighLevelControllerName initialControllerState;

   private final ContactableBodiesFactory<RobotSide> contactableBodiesFactory;

   private final HighLevelControllerParameters highLevelControllerParameters;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final HighLevelControlManagerFactory managerFactory;
   private final WholeBodyControllerCoreFactory controllerCoreFactory;
   private final PushRecoveryControlManagerFactory pushRecoveryManagerFactory;
   private final WalkingControllerParameters walkingControllerParameters;
   private final PushRecoveryControllerParameters pushRecoveryControllerParameters;
   private final ArrayList<Updatable> updatables = new ArrayList<>();
   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListenersToAttach = new ArrayList<>();
   private final ArrayList<ControllerFailureListener> controllerFailureListenersToAttach = new ArrayList<>();
   private final List<HighLevelHumanoidControllerPluginFactory> pluginFactories = new ArrayList<>();

   private final SideDependentList<String> footSensorNames;
   private final SideDependentList<String> wristSensorNames;

   private boolean createQueuedControllerCommandGenerator = false;
   private boolean createUserDesiredControllerCommandGenerator = false;
   private boolean useHeadingAndVelocityScript = true;

   private boolean isListeningToHighLevelStatePackets = true;

   private ConcurrentLinkedQueue<Command<?, ?>> controllerCommands;

   private HumanoidHighLevelControllerManager humanoidHighLevelControllerManager;

   public HighLevelHumanoidControllerFactory(ContactableBodiesFactory<RobotSide> contactableBodiesFactory,
                                             SideDependentList<String> footForceSensorNames,
                                             SideDependentList<String> wristSensorNames,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             WalkingControllerParameters walkingControllerParameters,
                                             PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                             CoPTrajectoryParameters copTrajectoryParameters)
   {
      this(contactableBodiesFactory,
           footForceSensorNames,
           wristSensorNames,
           highLevelControllerParameters,
           walkingControllerParameters,
           pushRecoveryControllerParameters,
           copTrajectoryParameters,
           new DefaultSplitFractionCalculatorParameters());
   }

   public HighLevelHumanoidControllerFactory(ContactableBodiesFactory<RobotSide> contactableBodiesFactory,
                                             SideDependentList<String> footForceSensorNames,
                                             SideDependentList<String> wristSensorNames,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             WalkingControllerParameters walkingControllerParameters,
                                             PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                             CoPTrajectoryParameters copTrajectoryParameters,
                                             SplitFractionCalculatorParametersReadOnly splitFractionCalculatorParameters)
   {
      this.highLevelControllerParameters = highLevelControllerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.pushRecoveryControllerParameters = pushRecoveryControllerParameters;
      this.contactableBodiesFactory = contactableBodiesFactory;
      this.footSensorNames = footForceSensorNames;
      this.wristSensorNames = wristSensorNames;

      commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
      try
      {
         commandInputManager.registerConversionHelper(new ClearDelayQueueConverter(ControllerAPIDefinition.getControllerSupportedCommands()));
      }
      catch (InstantiationException | IllegalAccessException e)
      {
         e.printStackTrace();
      }
      statusMessageOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

      managerFactory = new HighLevelControlManagerFactory(registry);
      managerFactory.setCopTrajectoryParameters(copTrajectoryParameters);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);
      managerFactory.setSplitFractionParameters(splitFractionCalculatorParameters);

      pushRecoveryManagerFactory = new PushRecoveryControlManagerFactory(managerFactory, registry);
      pushRecoveryManagerFactory.setCopTrajectoryParameters(copTrajectoryParameters);
      pushRecoveryManagerFactory.setWalkingControllerParameters(walkingControllerParameters);
      pushRecoveryManagerFactory.setPushRecoveryControllerParameters(pushRecoveryControllerParameters);

      controllerCoreFactory = new WholeBodyControllerCoreFactory(registry);
      controllerCoreFactory.setWalkingControllerParameters(walkingControllerParameters);
      controllerCoreFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
   }

   public void createInertialParameterManager(InertialEstimationParameters parameters)
   {
      managerFactory.setInertialEstimationParameters(parameters);
   }

   private ComponentBasedFootstepDataMessageGeneratorFactory componentBasedFootstepDataMessageGeneratorFactory;

   public void createComponentBasedFootstepDataMessageGenerator()
   {
      createComponentBasedFootstepDataMessageGenerator(false, null, null);
   }

   public void createComponentBasedFootstepDataMessageGenerator(boolean useHeadingAndVelocityScript,
                                                                HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters)
   {
      createComponentBasedFootstepDataMessageGenerator(useHeadingAndVelocityScript, null, headingAndVelocityEvaluationScriptParameters);
   }

   public void createComponentBasedFootstepDataMessageGenerator(boolean useHeadingAndVelocityScript,
                                                                FootstepAdjustment footstepAdjustment,
                                                                HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters)
   {
      if (componentBasedFootstepDataMessageGeneratorFactory != null)
         return;

      componentBasedFootstepDataMessageGeneratorFactory = new ComponentBasedFootstepDataMessageGeneratorFactory();
      componentBasedFootstepDataMessageGeneratorFactory.setRegistry();
      componentBasedFootstepDataMessageGeneratorFactory.setUseHeadingAndVelocityScript(useHeadingAndVelocityScript);
      componentBasedFootstepDataMessageGeneratorFactory.setHeadingAndVelocityEvaluationScriptParameters(headingAndVelocityEvaluationScriptParameters);
      componentBasedFootstepDataMessageGeneratorFactory.setFootStepAdjustment(footstepAdjustment);

      if (humanoidHighLevelControllerManager != null)
         humanoidHighLevelControllerManager.addControllerPluginFactory(componentBasedFootstepDataMessageGeneratorFactory);
      else
         pluginFactories.add(componentBasedFootstepDataMessageGeneratorFactory);
   }

   public void addControllerPlugin(HighLevelHumanoidControllerPluginFactory pluginFactory)
   {
      pluginFactories.add(pluginFactory);
   }

   private QueuedControllerCommandGenerator queuedControllerCommandGenerator;

   public void createQueuedControllerCommandGenerator(ConcurrentLinkedQueue<Command<?, ?>> controllerCommands)
   {
      if (queuedControllerCommandGenerator != null)
         return;

      if (controllerToolbox != null)
      {
         LogTools.info("Creating queued controller command generator");

         SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();
         CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
         double controlDT = controllerToolbox.getControlDT();
         queuedControllerCommandGenerator = new QueuedControllerCommandGenerator(controllerCommands,
                                                                                 commandInputManager,
                                                                                 statusMessageOutputManager,
                                                                                 walkingControllerParameters,
                                                                                 referenceFrames,
                                                                                 contactableFeet,
                                                                                 controlDT,
                                                                                 useHeadingAndVelocityScript,
                                                                                 registry);

         controllerToolbox.attachRobotMotionStatusChangedListener(queuedControllerCommandGenerator);
         controllerToolbox.addUpdatables(queuedControllerCommandGenerator.getModulesToUpdate());
      }
      else
      {
         createQueuedControllerCommandGenerator = true;
         this.controllerCommands = controllerCommands;
      }
   }

   private UserDesiredControllerCommandGenerators userDesiredControllerCommandGenerators = null;

   public void createUserDesiredControllerCommandGenerator()
   {
      if (userDesiredControllerCommandGenerators != null)
         return;

      if (controllerToolbox != null)
      {
         double defaultTrajectoryTime = 1.0;
         SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();
         userDesiredControllerCommandGenerators = new UserDesiredControllerCommandGenerators(commandInputManager,
                                                                                             controllerToolbox.getFullRobotModel(),
                                                                                             controllerToolbox.getReferenceFrames(),
                                                                                             contactableFeet,
                                                                                             walkingControllerParameters,
                                                                                             defaultTrajectoryTime,
                                                                                             registry);
      }
      else
      {
         createUserDesiredControllerCommandGenerator = true;
      }
   }

   public void useDefaultDiagnosticControlState()
   {
      //TODO
   }

   public void useDefaultCalibrationControlState()
   {
      //TODO
   }

   public void replaceControllerFactory(HighLevelControllerName controllerName, HighLevelControllerStateFactory controllerFactory)
   {
      if (controllerFactoriesMap.containsKey(controllerName))
         controllerStateFactories.remove(controllerFactoriesMap.get(controllerName));

      controllerStateFactories.add(controllerFactory);
      controllerFactoriesMap.put(controllerName, controllerFactory);
   }

   public void useDefaultDoNothingControlState()
   {
      DoNothingControllerStateFactory controllerStateFactory = new DoNothingControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.DO_NOTHING_BEHAVIOR, controllerStateFactory);
   }

   public void useDefaultStandPrepControlState()
   {
      StandPrepControllerStateFactory controllerStateFactory = new StandPrepControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.STAND_PREP_STATE, controllerStateFactory);
   }

   public void useDefaultStandReadyControlState()
   {
      StandReadyControllerStateFactory controllerStateFactory = new StandReadyControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.STAND_READY, controllerStateFactory);
   }

   public void useDefaultStandTransitionControlState()
   {
      useDefaultStandTransitionControlState(HighLevelControllerName.STAND_READY, HighLevelControllerName.WALKING);
   }

   public void useDefaultStandTransitionControlState(HighLevelControllerName startState, HighLevelControllerName endState)
   {
      StandTransitionControllerStateFactory controllerStateFactory = new StandTransitionControllerStateFactory(startState, endState);

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.STAND_TRANSITION_STATE, controllerStateFactory);
   }

   public void useDefaultWalkingControlState()
   {
      WalkingControllerStateFactory controllerStateFactory = new WalkingControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.WALKING, controllerStateFactory);
   }

   public void useDefaultPushRecoveryControlState()
   {
      PushRecoveryControllerStateFactory controllerStateFactory = new PushRecoveryControllerStateFactory(pushRecoveryManagerFactory);

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.PUSH_RECOVERY, controllerStateFactory);
   }

   public void useDefaultExitWalkingTransitionControlState(HighLevelControllerName targetState)
   {
      HighLevelControllerStateFactory controllerStateFactory = new ExitWalkingTransitionControllerStateFactory(targetState);

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.EXIT_WALKING, controllerStateFactory);
   }

   public void useDefaultFreezeControlState()
   {
      FreezeControllerStateFactory controllerStateFactory = new FreezeControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.FREEZE_STATE, controllerStateFactory);
   }

   public void useDefaultFallingControlState()
   {
      FallingControllerStateFactory controllerStateFactory = new FallingControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.FALLING_STATE, controllerStateFactory);
   }

   public void addCustomControlState(HighLevelControllerStateFactory customControllerStateFactory)
   {
      controllerStateFactories.add(customControllerStateFactory);
      controllerFactoriesMap.put(customControllerStateFactory.getStateEnum(), customControllerStateFactory);
   }

   /**
    * Adds a transition from {@code currentControlStateEnum} to {@code nextControlStateEnum} that will
    * trigger as soon as {@code currentControlStateEnum}'s
    * {@link HighLevelControllerState#isDone(double)} returns {@code true}.
    * 
    * @param currentControlStateEnum The state that is to be checked to see if it is finished.
    * @param nextControlStateEnum    The state to transition to.
    */
   public void addFinishedTransition(HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum)
   {
      stateTransitionFactories.add(new FinishedControllerStateTransitionFactory<>(currentControlStateEnum, nextControlStateEnum));
   }

   /**
    * Adds a transition from {@code currentControlStateEnum} to {@code nextControlStateEnum} that will
    * trigger as soon as {@code currentControlStateEnum}'s
    * {@link HighLevelControllerState#isDone(double)} returns {@code true}.
    * 
    * @param currentControlStateEnum The state that is to be checked to see if it is finished.
    * @param nextControlStateEnum    The state to transition to.
    * @param performNextStateOnEntry indicates whether {@link HighLevelControllerState#onEntry()} of
    *                                the next state be invoked (if {@code true}), or skipped
    *                                ({@code false}).
    */
   public void addFinishedTransition(HighLevelControllerName currentControlStateEnum,
                                     HighLevelControllerName nextControlStateEnum,
                                     boolean performNextStateOnEntry)
   {
      stateTransitionFactories.add(new FinishedControllerStateTransitionFactory<>(currentControlStateEnum, nextControlStateEnum, performNextStateOnEntry));
   }

   public void addRequestableTransition(HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum)
   {
      stateTransitionFactories.add(new RequestedControllerStateTransitionFactory<>(requestedHighLevelControllerState,
                                                                                   currentControlStateEnum,
                                                                                   nextControlStateEnum));
   }

   public void addControllerFailureTransition(HighLevelControllerName currentControlStateEnum, HighLevelControllerName fallbackControlStateEnum)
   {
      stateTransitionFactories.add(new ControllerFailedTransitionFactory(currentControlStateEnum, fallbackControlStateEnum));
   }

   public void addCustomStateTransition(ControllerStateTransitionFactory<HighLevelControllerName> stateTransitionFactory)
   {
      stateTransitionFactories.add(stateTransitionFactory);
   }

   public void setInitialState(HighLevelControllerName initialStateEnum)
   {
      this.initialControllerState = initialStateEnum;
   }

   public YoEnum<HighLevelControllerName> getRequestedControlStateEnum()
   {
      return requestedHighLevelControllerState;
   }

   public HumanoidHighLevelControllerManager getController(FullHumanoidRobotModel fullRobotModel,
                                                           double controlDT,
                                                           double gravity,
                                                           boolean kinematicsSimulation, // For fast non-physics preview simulations
                                                           YoDouble yoTime,
                                                           YoGraphicsListRegistry yoGraphicsListRegistry,
                                                           HumanoidRobotSensorInformation sensorInformation,
                                                           ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                                           CenterOfMassDataHolderReadOnly centerOfMassDataHolderForController,
                                                           CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                                           JointDesiredOutputListBasics lowLevelControllerOutput,
                                                           JointBasics... jointsToIgnore)
   {
      YoBoolean usingEstimatorCoMPosition = new YoBoolean("usingEstimatorCoMPosition", registry);
      YoBoolean usingEstimatorCoMVelocity = new YoBoolean("usingEstimatorCoMVelocity", registry);

      CenterOfMassStateProvider centerOfMassStateProvider = new CenterOfMassStateProvider()
      {
         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator(), worldFrame);

         @Override
         public void updateState()
         {
            centerOfMassJacobian.reset();
         }

         @Override
         public FramePoint3DReadOnly getCenterOfMassPosition()
         {
            usingEstimatorCoMPosition.set(centerOfMassDataHolderForController.hasCenterOfMassPosition());
            if (centerOfMassDataHolderForController.hasCenterOfMassPosition())
               return centerOfMassDataHolderForController.getCenterOfMassPosition();
            else
               return centerOfMassJacobian.getCenterOfMass();
         }

         @Override
         public FrameVector3DReadOnly getCenterOfMassVelocity()
         {
            usingEstimatorCoMVelocity.set(centerOfMassDataHolderForController.hasCenterOfMassVelocity());
            if (centerOfMassDataHolderForController.hasCenterOfMassVelocity())
               return centerOfMassDataHolderForController.getCenterOfMassVelocity();
            else
               return centerOfMassJacobian.getCenterOfMassVelocity();
         }
      };

      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel, centerOfMassStateProvider, null);

      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);
      SideDependentList<ContactableFoot> feet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      List<ContactablePlaneBody> additionalContacts = contactableBodiesFactory.createAdditionalContactPoints();
      contactableBodiesFactory.disposeFactory();

      List<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
         contactablePlaneBodies.add(feet.get(robotSide));
      contactablePlaneBodies.addAll(additionalContacts);

      double gravityZ = Math.abs(gravity);
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      double totalRobotWeight = totalMass * gravityZ;

      SideDependentList<FootSwitchInterface> footSwitches = createFootSwitches(feet,
                                                                               forceSensorDataHolder,
                                                                               fullRobotModel.getRootBody(),
                                                                               totalRobotWeight,
                                                                               kinematicsSimulation,
                                                                               yoGraphicsListRegistry,
                                                                               registry);
      SideDependentList<ForceSensorDataReadOnly> wristForceSensors = createWristForceSensors(forceSensorDataHolder);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the HighLevelHumanoidControllerToolbox /////////////////////////////////////////////
      double omega0 = walkingControllerParameters.getOmega0();
      controllerToolbox = new HighLevelHumanoidControllerToolbox(fullRobotModel,
                                                                 centerOfMassStateProvider,
                                                                 referenceFrames,
                                                                 footSwitches,
                                                                 wristForceSensors,
                                                                 yoTime,
                                                                 gravityZ,
                                                                 omega0,
                                                                 feet,
                                                                 controlDT,
                                                                 kinematicsSimulation,
                                                                 updatables,
                                                                 contactablePlaneBodies,
                                                                 yoGraphicsListRegistry,
                                                                 jointsToIgnore);
      controllerToolbox.attachControllerStateChangedListeners(controllerStateChangedListenersToAttach);
      attachControllerFailureListeners(controllerFailureListenersToAttach);
      if (createQueuedControllerCommandGenerator)
         createQueuedControllerCommandGenerator(controllerCommands);
      if (createUserDesiredControllerCommandGenerator)
         createUserDesiredControllerCommandGenerator();

      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      double defaultInitialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double defaultFinalTransferTime = walkingControllerParameters.getDefaultFinalTransferTime();
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime,
                                                                              defaultSwingTime,
                                                                              defaultInitialTransferTime,
                                                                              defaultFinalTransferTime,
                                                                              feet,
                                                                              statusMessageOutputManager,
                                                                              yoTime,
                                                                              yoGraphicsListRegistry,
                                                                              registry);
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);

      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      pushRecoveryManagerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      controllerCoreFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);

      ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver = controllerToolbox.getReferenceFrameHashCodeResolver();
      FrameMessageCommandConverter commandConversionHelper = new FrameMessageCommandConverter(referenceFrameHashCodeResolver);
      commandInputManager.registerConversionHelper(commandConversionHelper);

      humanoidHighLevelControllerManager = new HumanoidHighLevelControllerManager(commandInputManager,
                                                                                  statusMessageOutputManager,
                                                                                  initialControllerState,
                                                                                  highLevelControllerParameters,
                                                                                  walkingControllerParameters,
                                                                                  pushRecoveryControllerParameters,
                                                                                  requestedHighLevelControllerState,
                                                                                  controllerFactoriesMap,
                                                                                  stateTransitionFactories,
                                                                                  pluginFactories,
                                                                                  managerFactory,
                                                                                  controllerCoreFactory,
                                                                                  controllerToolbox,
                                                                                  centerOfPressureDataHolderForEstimator,
                                                                                  forceSensorDataHolder,
                                                                                  lowLevelControllerOutput);
      humanoidHighLevelControllerManager.addYoVariableRegistry(registry);
      humanoidHighLevelControllerManager.setListenToHighLevelStatePackets(isListeningToHighLevelStatePackets);
      for (RobotSide robotSide : RobotSide.values)
         humanoidHighLevelControllerManager.addYoGraphic(footSwitches.get(robotSide).getSCS2YoGraphics());
      return humanoidHighLevelControllerManager;
   }

   private SideDependentList<FootSwitchInterface> createFootSwitches(SideDependentList<? extends ContactablePlaneBody> bipedFeet,
                                                                     ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                                                     RigidBodyBasics rootBody,
                                                                     double totalRobotWeight,
                                                                     boolean kinematicsSimulation,
                                                                     YoGraphicsListRegistry yoGraphicsListRegistry,
                                                                     YoRegistry registry)
   {
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<>();

      if (kinematicsSimulation)  // For fast non-physics preview simulations, deal with there being no forces
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            SettableFootSwitch footSwitch = new SettableFootSwitch(bipedFeet.get(robotSide), totalRobotWeight, 2, registry);
            footSwitch.setFootContactState(true);
            footSwitches.put(robotSide, footSwitch);
         }
      }
      else
      {
         SideDependentList<FootSwitchFactory> footSwitchFactories = walkingControllerParameters.getFootSwitchFactories();

         for (RobotSide robotSide : RobotSide.values)
         {
            String footName = bipedFeet.get(robotSide).getName();
            ForceSensorDataReadOnly footForceSensor = forceSensorDataHolder.getData(footSensorNames.get(robotSide));
            FootSwitchFactory footSwitchFactory = footSwitchFactories.get(robotSide);
            FootSwitchInterface footSwitch = footSwitchFactory.newFootSwitch(footName,
                                                                             bipedFeet.get(robotSide),
                                                                             Collections.singleton(bipedFeet.get(robotSide.getOppositeSide())),
                                                                             rootBody,
                                                                             footForceSensor,
                                                                             totalRobotWeight,
                                                                             yoGraphicsListRegistry,
                                                                             registry);
            footSwitches.put(robotSide, footSwitch);
         }
      }

      return footSwitches;
   }

   private SideDependentList<ForceSensorDataReadOnly> createWristForceSensors(ForceSensorDataHolderReadOnly forceSensorDataHolder)
   {
      if (wristSensorNames == null)
         return null;

      SideDependentList<ForceSensorDataReadOnly> wristForceSensors = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         if (wristSensorNames.get(robotSide) == null)
         {
            return null;
         }
         ForceSensorDataReadOnly wristForceSensor = forceSensorDataHolder.getData(wristSensorNames.get(robotSide));
         wristForceSensors.put(robotSide, wristForceSensor);
      }
      return wristForceSensors;
   }

   public void addUpdatable(Updatable updatable)
   {
      if (controllerToolbox == null)
         this.updatables.add(updatable);
      else
         controllerToolbox.addUpdatable(updatable);
   }

   @Override
   public void closeAndDispose()
   {
      closeableAndDisposableRegistry.closeAndDispose();
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   public StatusMessageOutputManager getStatusOutputManager()
   {
      return statusMessageOutputManager;
   }

   public WholeBodyControllerCoreFactory getWholeBodyControllerCoreFactory()
   {
      return controllerCoreFactory;
   }

   public void attachControllerFailureListeners(List<ControllerFailureListener> listeners)
   {
      for (int i = 0; i < listeners.size(); i++)
         attachControllerFailureListener(listeners.get(i));
   }

   public void attachControllerFailureListener(ControllerFailureListener listener)
   {
      if (controllerToolbox != null)
         controllerToolbox.attachControllerFailureListener(listener);
      else
         controllerFailureListenersToAttach.add(listener);
   }

   public boolean detachControllerFailureListener(ControllerFailureListener listener)
   {
      if (controllerToolbox != null)
         return controllerToolbox.detachControllerFailureListener(listener);
      else
         return controllerFailureListenersToAttach.remove(listener);
   }

   public void attachControllerStateChangedListeners(List<ControllerStateChangedListener> listeners)
   {
      for (int i = 0; i < listeners.size(); i++)
         attachControllerStateChangedListener(listeners.get(i));
   }

   public void attachControllerStateChangedListener(ControllerStateChangedListener listener)
   {
      if (controllerToolbox != null)
         controllerToolbox.attachControllerStateChangedListener(listener);
      else
         controllerStateChangedListenersToAttach.add(listener);
   }

   public void attachRobotMotionStatusChangedListener(RobotMotionStatusChangedListener listener)
   {
      controllerToolbox.attachRobotMotionStatusChangedListener(listener);
   }

   public void createControllerNetworkSubscriber(String robotName, RealtimeROS2Node realtimeROS2Node)
   {
      ROS2Topic<?> inputTopic = HumanoidControllerAPI.getInputTopic(robotName);
      ROS2Topic<?> outputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(inputTopic,
                                                                                                commandInputManager,
                                                                                                outputTopic,
                                                                                                statusMessageOutputManager,
                                                                                                realtimeROS2Node);

      controllerNetworkSubscriber.registerSubcriberWithMessageUnpacker(WholeBodyTrajectoryMessage.class,
                                                                       9,
                                                                       MessageUnpackingTools.createWholeBodyTrajectoryMessageUnpacker());
      controllerNetworkSubscriber.registerSubcriberWithMessageUnpacker(WholeBodyStreamingMessage.class,
                                                                       9,
                                                                       MessageUnpackingTools.createWholeBodyStreamingMessageUnpacker());
      controllerNetworkSubscriber.addMessageCollectors(ControllerAPIDefinition.createDefaultMessageIDExtractor(), 3);
      controllerNetworkSubscriber.addMessageValidator(ControllerAPIDefinition.createDefaultMessageValidation());
   }

   public HighLevelHumanoidControllerToolbox getHighLevelHumanoidControllerToolbox()
   {
      return controllerToolbox;
   }

   public HighLevelControllerName getCurrentHighLevelControlState()
   {
      return humanoidHighLevelControllerManager.getCurrentHighLevelControlState();
   }

   public void addHighLevelStateChangedListener(StateChangedListener<HighLevelControllerName> stateChangedListener)
   {
      humanoidHighLevelControllerManager.addHighLevelStateChangedListener(stateChangedListener);
   }

   public void setListenToHighLevelStatePackets(boolean isListening)
   {
      if (humanoidHighLevelControllerManager != null)
         humanoidHighLevelControllerManager.setListenToHighLevelStatePackets(isListening);
      else
         isListeningToHighLevelStatePackets = isListening;
   }

}
