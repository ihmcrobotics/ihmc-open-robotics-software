package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired.UserDesiredControllerCommandGenerators;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ComponentBasedFootstepDataMessageGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.QueuedControllerCommandGenerator;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HumanoidHighLevelControllerManager;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.KinematicsBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchAndContactSensorFusedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameMessageCommandConverter;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.*;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class HighLevelHumanoidControllerFactory implements CloseableAndDisposable
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   private final ArrayList<HighLevelControllerStateFactory> controllerStateFactories = new ArrayList<>();
   private final EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerFactoriesMap = new EnumMap<>(HighLevelControllerName.class);

   private final ArrayList<ControllerStateTransitionFactory<HighLevelControllerName>> stateTransitionFactories = new ArrayList<>();

   private HighLevelHumanoidControllerToolbox controllerToolbox = null;

   private final YoEnum<HighLevelControllerName> requestedHighLevelControllerState = new YoEnum<>("requestedHighLevelControllerState", registry,
                                                                                                  HighLevelControllerName.class, true);
   private HighLevelControllerName initialControllerState;

   private final ContactableBodiesFactory contactableBodiesFactory;

   private final HighLevelControllerParameters highLevelControllerParameters;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final HighLevelControlManagerFactory managerFactory;
   private final WalkingControllerParameters walkingControllerParameters;
   private final ICPWithTimeFreezingPlannerParameters icpPlannerParameters;
   private final JumpControllerParameters jumpControllerParameters;
   private final JumpControlManagerFactory jumpControlManagerFactory;

   private final ArrayList<Updatable> updatables = new ArrayList<>();
   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListenersToAttach = new ArrayList<>();
   private final ArrayList<ControllerFailureListener> controllerFailureListenersToAttach = new ArrayList<>();

   private final SideDependentList<String> footSensorNames;
   private final SideDependentList<String> footContactSensorNames;
   private final SideDependentList<String> wristSensorNames;

   private HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters;
   private boolean createComponentBasedFootstepDataMessageGenerator = false;
   private boolean createQueuedControllerCommandGenerator = false;
   private boolean createUserDesiredControllerCommandGenerator = true;
   private boolean useHeadingAndVelocityScript = true;
   private HeightMap heightMapForFootstepZ = null;

   private boolean isListeningToHighLevelStatePackets = true;

   private ConcurrentLinkedQueue<Command<?, ?>> controllerCommands;

   private HumanoidHighLevelControllerManager humanoidHighLevelControllerManager;

   public HighLevelHumanoidControllerFactory(ContactableBodiesFactory contactableBodiesFactory, SideDependentList<String> footForceSensorNames,
                                             SideDependentList<String> footContactSensorNames, SideDependentList<String> wristSensorNames,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             WalkingControllerParameters walkingControllerParameters, ICPWithTimeFreezingPlannerParameters icpPlannerParameters)
   {
      this(contactableBodiesFactory, footForceSensorNames, footContactSensorNames, wristSensorNames, highLevelControllerParameters, walkingControllerParameters,
           icpPlannerParameters, null);
   }

   public HighLevelHumanoidControllerFactory(ContactableBodiesFactory contactableBodiesFactory, SideDependentList<String> footForceSensorNames,
                                             SideDependentList<String> footContactSensorNames, SideDependentList<String> wristSensorNames,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             WalkingControllerParameters walkingControllerParameters, ICPWithTimeFreezingPlannerParameters icpPlannerParameters,
                                             JumpControllerParameters jumpControllerParameteres)
   {
      this.highLevelControllerParameters = highLevelControllerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.icpPlannerParameters = icpPlannerParameters;
      this.jumpControllerParameters = jumpControllerParameteres;
      this.contactableBodiesFactory = contactableBodiesFactory;
      this.footSensorNames = footForceSensorNames;
      this.footContactSensorNames = footContactSensorNames;
      this.wristSensorNames = wristSensorNames;

      commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
      statusMessageOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

      managerFactory = new HighLevelControlManagerFactory(statusMessageOutputManager, registry);
      managerFactory.setCapturePointPlannerParameters(icpPlannerParameters);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);

      jumpControlManagerFactory = new JumpControlManagerFactory(registry);
   }

   private ComponentBasedFootstepDataMessageGenerator footstepGenerator;

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
      if (footstepGenerator != null)
         return;

      if (controllerToolbox != null)
      {
         SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();
         CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
         double controlDT = controllerToolbox.getControlDT();
         ComponentBasedFootstepDataMessageGenerator footstepGenerator = new ComponentBasedFootstepDataMessageGenerator(commandInputManager,
                                                                                                                       statusMessageOutputManager,
                                                                                                                       walkingControllerParameters,
                                                                                                                       headingAndVelocityEvaluationScriptParameters,
                                                                                                                       referenceFrames, contactableFeet,
                                                                                                                       controlDT, useHeadingAndVelocityScript,
                                                                                                                       heightMapForFootstepZ, registry);
         controllerToolbox.addUpdatable(footstepGenerator);
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
         System.out.println("In createdQueuedControllerCommandGenerator");

         SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();
         CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
         double controlDT = controllerToolbox.getControlDT();
         queuedControllerCommandGenerator = new QueuedControllerCommandGenerator(controllerCommands, commandInputManager, statusMessageOutputManager,
                                                                                 walkingControllerParameters, referenceFrames, contactableFeet, controlDT,
                                                                                 useHeadingAndVelocityScript, registry);

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

   public void useDefaultFreezeControlState()
   {
      FreezeControllerStateFactory controllerStateFactory = new FreezeControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.FREEZE_STATE, controllerStateFactory);
   }

   public void useDefaultJumpControllerState()
   {
      JumpControllerStateFactory controllerStateFactory = new JumpControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(HighLevelControllerName.JUMPING, controllerStateFactory);
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
                                        ForceSensorDataHolderReadOnly forceSensorDataHolder, CenterOfMassDataHolderReadOnly centerOfMassDataHolder,
                                        ContactSensorHolder contactSensorHolder, CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                        JointDesiredOutputList lowLevelControllerOutput, InverseDynamicsJoint... jointsToIgnore)
   {
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      SideDependentList<ContactableFoot> feet = contactableBodiesFactory.createFootContactableBodies(fullRobotModel, referenceFrames);
      List<ContactablePlaneBody> additionalContacts = contactableBodiesFactory.createAdditionalContactPoints(fullRobotModel);
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
      controllerToolbox = new HighLevelHumanoidControllerToolbox(fullRobotModel, referenceFrames, footSwitches, centerOfMassDataHolder, wristForceSensors,
                                                                 yoTime, gravityZ, omega0, feet, controlDT, updatables, contactablePlaneBodies,
                                                                 yoGraphicsListRegistry, jointsToIgnore);
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
      jumpControlManagerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      
      ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver = controllerToolbox.getReferenceFrameHashCodeResolver();
      FrameMessageCommandConverter commandConversionHelper = new FrameMessageCommandConverter(referenceFrameHashCodeResolver);
      commandInputManager.registerConversionHelper(commandConversionHelper);

      humanoidHighLevelControllerManager = new HumanoidHighLevelControllerManager(commandInputManager, statusMessageOutputManager, initialControllerState,
                                                                                  highLevelControllerParameters, walkingControllerParameters,
                                                                                  icpPlannerParameters, jumpControllerParameters, requestedHighLevelControllerState,
                                                                                  controllerFactoriesMap, stateTransitionFactories, managerFactory, jumpControlManagerFactory,
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

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSwitchInterface footSwitch = null;
         String footName = bipedFeet.get(robotSide).getName();
         ForceSensorDataReadOnly footForceSensor = forceSensorDataHolder.getByName(footSensorNames.get(robotSide));
         double contactThresholdForce = walkingControllerParameters.getContactThresholdForce();
         double footSwitchCoPThresholdFraction = walkingControllerParameters.getCoPThresholdFraction();

         switch (walkingControllerParameters.getFootSwitchType())
         {
         case KinematicBased:
            footSwitch = new KinematicsBasedFootSwitch(footName, bipedFeet, walkingControllerParameters.getContactThresholdHeight(), totalRobotWeight,
                                                       robotSide, registry); //controller switch doesnt need com
            break;

         case WrenchBased:
            WrenchBasedFootSwitch wrenchBasedFootSwitch = new WrenchBasedFootSwitch(footName, footForceSensor, footSwitchCoPThresholdFraction, totalRobotWeight,
                                                                                    bipedFeet.get(robotSide), yoGraphicsListRegistry, contactThresholdForce,
                                                                                    registry);
            wrenchBasedFootSwitch.setSecondContactThresholdForce(walkingControllerParameters.getSecondContactThresholdForceIgnoringCoP());
            footSwitch = wrenchBasedFootSwitch;
            break;

         case WrenchAndContactSensorFused:
            footSwitch = new WrenchAndContactSensorFusedFootSwitch(footName, footForceSensor,
                                                                   contactSensorHolder.getByName(footContactSensorNames.get(robotSide)),
                                                                   footSwitchCoPThresholdFraction, totalRobotWeight, bipedFeet.get(robotSide),
                                                                   yoGraphicsListRegistry, contactThresholdForce, registry);
            break;
         }

         assert footSwitch != null;
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
      this.updatables.add(updatable);
   }

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

   public void setInverseDynamicsCalculatorListener(InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener)
   {
      controllerToolbox.setInverseDynamicsCalculatorListener(inverseDynamicsCalculatorListener);
   }

   public void attachRobotMotionStatusChangedListener(RobotMotionStatusChangedListener listener)
   {
      controllerToolbox.attachRobotMotionStatusChangedListener(listener);
   }

   public void createControllerNetworkSubscriber(PeriodicThreadScheduler scheduler, PacketCommunicator packetCommunicator)
   {
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(commandInputManager, statusMessageOutputManager, scheduler,
                                                                                                packetCommunicator);
      closeableAndDisposableRegistry.registerCloseableAndDisposable(controllerNetworkSubscriber);
   }

   public HighLevelHumanoidControllerToolbox getHighLevelHumanoidControllerToolbox()
   {
      return controllerToolbox;
   }

   public HighLevelControllerName getCurrentHighLevelControlState()
   {
      return humanoidHighLevelControllerManager.getCurrentHighLevelControlState();
   }

   public void reinitializeWalking(boolean keepPosition)
   {
      humanoidHighLevelControllerManager.requestHighLevelControllerState(HighLevelControllerName.WALKING);
      if (keepPosition)
         humanoidHighLevelControllerManager.reinitializeWalking();
   }

   public void setListenToHighLevelStatePackets(boolean isListening)
   {
      if (humanoidHighLevelControllerManager != null)
         humanoidHighLevelControllerManager.setListenToHighLevelStatePackets(isListening);
      else
         isListeningToHighLevelStatePackets = isListening;
   }

}
