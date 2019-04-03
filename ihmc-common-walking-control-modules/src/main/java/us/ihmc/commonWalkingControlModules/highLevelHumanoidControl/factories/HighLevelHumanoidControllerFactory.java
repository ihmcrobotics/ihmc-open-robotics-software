package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired.UserDesiredControllerCommandGenerators;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.QueuedControllerCommandGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScript;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HumanoidHighLevelControllerManager;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.ROS2ModuleIdentifier;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.ClearDelayQueueConverter;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameMessageCommandConverter;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.robotics.sensors.ContactSensor;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class HighLevelHumanoidControllerFactory implements CloseableAndDisposable
{
   public static final ROS2ModuleIdentifier ROS2_ID = new ROS2ModuleIdentifier("ihmc_controller", ROS2Tools.HUMANOID_CONTROL_MODULE);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   private final ArrayList<HighLevelControllerStateFactory> controllerStateFactories = new ArrayList<>();
   private final EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerFactoriesMap = new EnumMap<>(HighLevelControllerName.class);

   private final ArrayList<ControllerStateTransitionFactory<HighLevelControllerName>> stateTransitionFactories = new ArrayList<>();

   private HighLevelHumanoidControllerToolbox controllerToolbox = null;

   private final YoEnum<HighLevelControllerName> requestedHighLevelControllerState = new YoEnum<>("requestedHighLevelControllerState", registry,
                                                                                                  HighLevelControllerName.class, true);
   private HighLevelControllerName initialControllerState;

   private final ContactableBodiesFactory<RobotSide> contactableBodiesFactory;

   private final HighLevelControllerParameters highLevelControllerParameters;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final HighLevelControlManagerFactory managerFactory;
   private final WalkingControllerParameters walkingControllerParameters;
   private final ICPWithTimeFreezingPlannerParameters icpPlannerParameters;

   private final ArrayList<Updatable> updatables = new ArrayList<>();
   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListenersToAttach = new ArrayList<>();
   private final ArrayList<ControllerFailureListener> controllerFailureListenersToAttach = new ArrayList<>();

   private final SideDependentList<String> footSensorNames;
   private final SideDependentList<String> footContactSensorNames;
   private final SideDependentList<String> wristSensorNames;

   private YoGraphicsListRegistry yoGraphicsListRegistry;

   private HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters;
   private boolean createComponentBasedFootstepDataMessageGenerator = false;
   private boolean createQueuedControllerCommandGenerator = false;
   private boolean createUserDesiredControllerCommandGenerator = true;
   private boolean useHeadingAndVelocityScript = true;
   private HeightMap heightMapForFootstepZ = null;

   private boolean isListeningToHighLevelStatePackets = true;

   private ConcurrentLinkedQueue<Command<?, ?>> controllerCommands;

   private HumanoidHighLevelControllerManager humanoidHighLevelControllerManager;

   public HighLevelHumanoidControllerFactory(ContactableBodiesFactory<RobotSide> contactableBodiesFactory, SideDependentList<String> footForceSensorNames,
                                             SideDependentList<String> footContactSensorNames, SideDependentList<String> wristSensorNames,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             WalkingControllerParameters walkingControllerParameters, ICPWithTimeFreezingPlannerParameters icpPlannerParameters)
   {
      this.highLevelControllerParameters = highLevelControllerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.icpPlannerParameters = icpPlannerParameters;
      this.contactableBodiesFactory = contactableBodiesFactory;
      this.footSensorNames = footForceSensorNames;
      this.footContactSensorNames = footContactSensorNames;
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

      managerFactory = new HighLevelControlManagerFactory(statusMessageOutputManager, registry);
      managerFactory.setCapturePointPlannerParameters(icpPlannerParameters);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);
   }

   private ContinuousStepGenerator continuousStepGenerator;

   public void createComponentBasedFootstepDataMessageGenerator()
   {
      createComponentBasedFootstepDataMessageGenerator(false, null);
   }

   public void createComponentBasedFootstepDataMessageGenerator(boolean useHeadingAndVelocityScript)
   {
      createComponentBasedFootstepDataMessageGenerator(useHeadingAndVelocityScript, null);
   }

   public void createComponentBasedFootstepDataMessageGenerator(boolean useHeadingAndVelocityScript, HeightMap heightMapForFootstepZ)
   {
      if (continuousStepGenerator != null)
         return;

      if (controllerToolbox != null)
      {
         CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
         double controlDT = controllerToolbox.getControlDT();
         continuousStepGenerator = new ContinuousStepGenerator(registry);
         continuousStepGenerator.setFootstepStatusListener(statusMessageOutputManager);
         continuousStepGenerator.setFrameBasedFootPoseProvider(referenceFrames.getSoleZUpFrames());
         continuousStepGenerator.configureWith(walkingControllerParameters);
         continuousStepGenerator.setFootstepMessenger(commandInputManager::submitMessage);
         if (yoGraphicsListRegistry != null)
            continuousStepGenerator.setupVisualization(controllerToolbox.getContactableFeet(), yoGraphicsListRegistry);
         if (heightMapForFootstepZ != null)
            continuousStepGenerator.setHeightMapBasedFootstepAdjustment(heightMapForFootstepZ);

         if (useHeadingAndVelocityScript)
         {
            HeadingAndVelocityEvaluationScript script = new HeadingAndVelocityEvaluationScript(controlDT, controllerToolbox.getYoTime(),
                                                                                               headingAndVelocityEvaluationScriptParameters, registry);
            continuousStepGenerator.setDesiredTurningVelocityProvider(script.getDesiredTurningVelocityProvider());
            continuousStepGenerator.setDesiredVelocityProvider(script.getDesiredVelocityProvider());
            controllerToolbox.addUpdatable(script);
         }
         else
         {
            continuousStepGenerator.setYoComponentProviders();
         }
         controllerToolbox.addUpdatable(continuousStepGenerator);
      }
      else
      {
         createComponentBasedFootstepDataMessageGenerator = true;
         this.useHeadingAndVelocityScript = useHeadingAndVelocityScript;
         this.heightMapForFootstepZ = heightMapForFootstepZ;
      }
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
         queuedControllerCommandGenerator = new QueuedControllerCommandGenerator(controllerCommands, commandInputManager, statusMessageOutputManager,
                                                                                 walkingControllerParameters, referenceFrames, contactableFeet, controlDT,
                                                                                 useHeadingAndVelocityScript, registry);

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
         userDesiredControllerCommandGenerators = new UserDesiredControllerCommandGenerators(commandInputManager, controllerToolbox.getFullRobotModel(),
                                                                                             controllerToolbox.getReferenceFrames(), contactableFeet,
                                                                                             walkingControllerParameters, defaultTrajectoryTime, registry);
      }
      else
      {
         createUserDesiredControllerCommandGenerator = true;
      }
   }

   public void setHeadingAndVelocityEvaluationScriptParameters(HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      headingAndVelocityEvaluationScriptParameters = walkingScriptParameters;
   }

   public void useDefaultDiagnosticControlState()
   {
      //TODO
   }

   public void useDefaultCalibrationControlState()
   {
      //TODO
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
      StandTransitionControllerStateFactory controllerStateFactory = new StandTransitionControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.STAND_TRANSITION_STATE, controllerStateFactory);
   }

   public void useDefaultWalkingControlState()
   {
      WalkingControllerStateFactory controllerStateFactory = new WalkingControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.WALKING, controllerStateFactory);
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

   public void addCustomControlState(HighLevelControllerStateFactory customControllerStateFactory)
   {
      controllerStateFactories.add(customControllerStateFactory);
      controllerFactoriesMap.put(customControllerStateFactory.getStateEnum(), customControllerStateFactory);
   }

   public void addFinishedTransition(HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum)
   {
      stateTransitionFactories.add(new FinishedControllerStateTransitionFactory<>(currentControlStateEnum, nextControlStateEnum));
   }

   public void addRequestableTransition(HighLevelControllerName currentControlStateEnum, HighLevelControllerName nextControlStateEnum)
   {
      stateTransitionFactories.add(new RequestedControllerStateTransitionFactory<>(requestedHighLevelControllerState, currentControlStateEnum,
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

   public RobotController getController(FullHumanoidRobotModel fullRobotModel, double controlDT, double gravity, YoDouble yoTime,
                                        YoGraphicsListRegistry yoGraphicsListRegistry, DRCRobotSensorInformation sensorInformation,
                                        ForceSensorDataHolderReadOnly forceSensorDataHolder, ContactSensorHolder contactSensorHolder,
                                        CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator, JointDesiredOutputList lowLevelControllerOutput,
                                        JointBasics... jointsToIgnore)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

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

      SideDependentList<FootSwitchInterface> footSwitches = createFootSwitches(feet, forceSensorDataHolder, contactSensorHolder, totalRobotWeight,
                                                                               yoGraphicsListRegistry, registry);
      SideDependentList<ForceSensorDataReadOnly> wristForceSensors = createWristForceSensors(forceSensorDataHolder);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the HighLevelHumanoidControllerToolbox /////////////////////////////////////////////
      double omega0 = walkingControllerParameters.getOmega0();
      controllerToolbox = new HighLevelHumanoidControllerToolbox(fullRobotModel, referenceFrames, footSwitches, wristForceSensors, yoTime, gravityZ, omega0,
                                                                 feet, controlDT, updatables, contactablePlaneBodies, yoGraphicsListRegistry, jointsToIgnore);
      controllerToolbox.attachControllerStateChangedListeners(controllerStateChangedListenersToAttach);
      attachControllerFailureListeners(controllerFailureListenersToAttach);
      if (createComponentBasedFootstepDataMessageGenerator)
         createComponentBasedFootstepDataMessageGenerator(useHeadingAndVelocityScript, heightMapForFootstepZ);
      if (createUserDesiredControllerCommandGenerator)
         if (createQueuedControllerCommandGenerator)
            createQueuedControllerCommandGenerator(controllerCommands);
      if (createUserDesiredControllerCommandGenerator)
         createUserDesiredControllerCommandGenerator();

      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      double defaultTouchdownTime = walkingControllerParameters.getDefaultTouchdownTime();
      double defaultInitialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double defaultFinalTransferTime = walkingControllerParameters.getDefaultFinalTransferTime();
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime, defaultSwingTime, defaultTouchdownTime,
                                                                              defaultInitialTransferTime, defaultFinalTransferTime, feet,
                                                                              statusMessageOutputManager, yoTime, yoGraphicsListRegistry, registry);
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);

      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);

      ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver = controllerToolbox.getReferenceFrameHashCodeResolver();
      FrameMessageCommandConverter commandConversionHelper = new FrameMessageCommandConverter(referenceFrameHashCodeResolver);
      commandInputManager.registerConversionHelper(commandConversionHelper);

      humanoidHighLevelControllerManager = new HumanoidHighLevelControllerManager(commandInputManager, statusMessageOutputManager, initialControllerState,
                                                                                  highLevelControllerParameters, walkingControllerParameters,
                                                                                  icpPlannerParameters, requestedHighLevelControllerState,
                                                                                  controllerFactoriesMap, stateTransitionFactories, managerFactory,
                                                                                  controllerToolbox, centerOfPressureDataHolderForEstimator,
                                                                                  forceSensorDataHolder, lowLevelControllerOutput);
      humanoidHighLevelControllerManager.addYoVariableRegistry(registry);
      humanoidHighLevelControllerManager.setListenToHighLevelStatePackets(isListeningToHighLevelStatePackets);
      return humanoidHighLevelControllerManager;
   }

   private SideDependentList<FootSwitchInterface> createFootSwitches(SideDependentList<? extends ContactablePlaneBody> bipedFeet,
                                                                     ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                                                     ContactSensorHolder contactSensorHolder, double totalRobotWeight,
                                                                     YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<FootSwitchInterface>();

      FootSwitchFactory footSwitchFactory = walkingControllerParameters.getFootSwitchFactory();

      for (RobotSide robotSide : RobotSide.values)
      {
         String footName = bipedFeet.get(robotSide).getName();
         ForceSensorDataReadOnly footForceSensor = forceSensorDataHolder.getByName(footSensorNames.get(robotSide));
         ContactSensor footContactSensor = contactSensorHolder == null ? null : contactSensorHolder.getByName(footContactSensorNames.get(robotSide));
         FootSwitchInterface footSwitch = footSwitchFactory.newFootSwitch(footName, bipedFeet.get(robotSide),
                                                                          Collections.singleton(bipedFeet.get(robotSide.getOppositeSide())), footForceSensor,
                                                                          footContactSensor, totalRobotWeight, yoGraphicsListRegistry, registry);
         footSwitches.put(robotSide, footSwitch);
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
         ForceSensorDataReadOnly wristForceSensor = forceSensorDataHolder.getByName(wristSensorNames.get(robotSide));
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

   public void createControllerNetworkSubscriber(String robotName, RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      ROS2Tools.MessageTopicNameGenerator publisherTopicNameGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(subscriberTopicNameGenerator, commandInputManager,
                                                                                                publisherTopicNameGenerator, statusMessageOutputManager,
                                                                                                realtimeRos2Node);

      controllerNetworkSubscriber.registerSubcriberWithMessageUnpacker(WholeBodyTrajectoryMessage.class, 9,
                                                                       MessageUnpackingTools.createWholeBodyTrajectoryMessageUnpacker());
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

   public void setListenToHighLevelStatePackets(boolean isListening)
   {
      if (humanoidHighLevelControllerManager != null)
         humanoidHighLevelControllerManager.setListenToHighLevelStatePackets(isListening);
      else
         isListeningToHighLevelStatePackets = isListening;
   }

}
