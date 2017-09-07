package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired.UserDesiredControllerCommandGenerators;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ComponentBasedFootstepDataMessageGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.QueuedControllerCommandGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.NewHumanoidHighLevelControllerManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.KinematicsBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchAndContactSensorFusedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameMessageCommandConverter;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.*;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

public class HighLevelHumanoidControllerFactory implements CloseableAndDisposable
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();


   private final ArrayList<HighLevelControllerStateFactory> controllerStateFactories = new ArrayList<>();
   private final EnumMap<NewHighLevelControllerStates, HighLevelControllerStateFactory> controllerFactoriesMap = new EnumMap<>(NewHighLevelControllerStates.class);

   private final ArrayList<ControllerStateTransitionFactory<NewHighLevelControllerStates>> stateTransitionFactories = new ArrayList<>();

   private HighLevelHumanoidControllerToolbox controllerToolbox = null;

   private final YoEnum<NewHighLevelControllerStates> requestedHighLevelControllerState = new YoEnum<>("requestedHighLevelControllerState", registry,
                                                                                                                                   NewHighLevelControllerStates.class, true);
   private NewHighLevelControllerStates initialControllerState;

   private final ContactableBodiesFactory contactableBodiesFactory;

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

   private HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters;
   private boolean createComponentBasedFootstepDataMessageGenerator = false;
   private boolean createQueuedControllerCommandGenerator = false;
   private boolean createUserDesiredControllerCommandGenerator = true;
   private boolean useHeadingAndVelocityScript = true;
   private HeightMap heightMapForFootstepZ = null;

   private ConcurrentLinkedQueue<Command<?, ?>> controllerCommands;

   private boolean setupInverseDynamicsSolver = true;
   private boolean setupInverseKinematicsSolver = false;
   private boolean setupVirtualModelControlSolver = false;

   public HighLevelHumanoidControllerFactory(ContactableBodiesFactory contactableBodiesFactory, SideDependentList<String> footForceSensorNames,
                                             SideDependentList<String> footContactSensorNames, SideDependentList<String> wristSensorNames,
                                             HighLevelControllerParameters highLevelControllerParameters, WalkingControllerParameters walkingControllerParameters,
                                             ICPWithTimeFreezingPlannerParameters icpPlannerParameters)
   {
      this.highLevelControllerParameters = highLevelControllerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.icpPlannerParameters = icpPlannerParameters;
      this.contactableBodiesFactory = contactableBodiesFactory;
      this.footSensorNames = footForceSensorNames;
      this.footContactSensorNames = footContactSensorNames;
      this.wristSensorNames = wristSensorNames;

      commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
      statusMessageOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

      managerFactory = new HighLevelControlManagerFactory(statusMessageOutputManager, registry);
      managerFactory.setCapturePointPlannerParameters(icpPlannerParameters);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);
   }

   /**
    * Specifies whether the inverse dynamics module of the {@link WholeBodyControllerCore} should be created or not.
    * <p>
    * This module is created by default as the {@link WalkingHighLevelHumanoidController} needs it.
    * </p>
    *
    * @param setup whether to setup the inverse dynamics mode or not.
    */
   public void setupControllerCoreInverseDynamicsMode(boolean setup)
   {
      setupInverseDynamicsSolver = setup;
   }

   /**
    * Specifies whether the inverse kinematics module of the {@link WholeBodyControllerCore} should be created or not.
    * <p>
    * This module is not created by default to prevent creating unused {@link YoVariable}s.
    * </p>
    *
    * @param setup whether to setup the inverse kinematics mode or not.
    */
   public void setupControllerCoreInverseKinematicsMode(boolean setup)
   {
      setupInverseKinematicsSolver = setup;
   }

   /**
    * Specifies whether the virtual model control module of the {@link WholeBodyControllerCore} should be created or not.
    * <p>
    * This module is not created by default to prevent creating unused {@link YoVariable}s.
    * </p>
    *
    * @param setup whether to setup the virtual model control mode or not.
    */
   public void setupControllerCoreVirtualModelControlMode(boolean setup)
   {
      setupVirtualModelControlSolver = setup;
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
         ComponentBasedFootstepDataMessageGenerator footstepGenerator = new ComponentBasedFootstepDataMessageGenerator(commandInputManager, statusMessageOutputManager,
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
      NewDoNothingControllerStateFactory controllerStateFactory = new NewDoNothingControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(NewHighLevelControllerStates.DO_NOTHING_STATE, controllerStateFactory);
   }

   public void useDefaultStandPrepControlState()
   {
      StandPrepControllerStateFactory controllerStateFactory = new StandPrepControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(NewHighLevelControllerStates.STAND_PREP_STATE, controllerStateFactory);
   }

   public void useDefaultStandReadyControlState()
   {
      StandReadyControllerStateFactory controllerStateFactory = new StandReadyControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(NewHighLevelControllerStates.STAND_READY, controllerStateFactory);
   }

   public void useDefaultStandTransitionControlState()
   {
      StandTransitionControllerStateFactory controllerStateFactory = new StandTransitionControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(NewHighLevelControllerStates.STAND_TRANSITION_STATE, controllerStateFactory);
   }

   public void useDefaultWalkingControlState()
   {
      NewWalkingControllerStateFactory controllerStateFactory = new NewWalkingControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(NewHighLevelControllerStates.WALKING_STATE, controllerStateFactory);
   }

   public void useDefaultFreezeControlState()
   {
      FreezeControllerStateFactory controllerStateFactory = new FreezeControllerStateFactory();

      controllerStateFactories.add(controllerStateFactory);
      controllerFactoriesMap.put(NewHighLevelControllerStates.FREEZE_STATE, controllerStateFactory);
   }

   public void addCustomControlState(HighLevelControllerStateFactory customControllerStateFactory)
   {
      controllerStateFactories.add(customControllerStateFactory);
      controllerFactoriesMap.put(customControllerStateFactory.getStateEnum(), customControllerStateFactory);
   }

   public void addFinishedTransition(NewHighLevelControllerStates currentControlStateEnum, NewHighLevelControllerStates nextControlStateEnum)
   {
      stateTransitionFactories.add(new FinishedControllerStateTransitionFactory<>(currentControlStateEnum, nextControlStateEnum));
   }

   public void addRequestableTransition(YoEnum<NewHighLevelControllerStates> requestedControlState, NewHighLevelControllerStates currentControlStateEnum,
                                        NewHighLevelControllerStates nextControlStateEnum)
   {
      stateTransitionFactories.add(new RequestedControllerStateTransitionFactory<>(requestedControlState, currentControlStateEnum, nextControlStateEnum));
   }

   public void addControllerFailureTransition(ControllerFailureListener controllerFailureListener, NewHighLevelControllerStates currentControlStateEnum,
                                              NewHighLevelControllerStates fallbackControlStateEnum)
   {
      //TODO
   }

   public void addCustomStateTransition(ControllerStateTransitionFactory<NewHighLevelControllerStates> stateTransitionFactory)
   {
      stateTransitionFactories.add(stateTransitionFactory);
   }

   public void setInitialState(NewHighLevelControllerStates initialStateEnum)
   {
      this.initialControllerState = initialStateEnum;
   }

   public YoEnum<NewHighLevelControllerStates> getRequestedControlStateEnum()
   {
      return requestedHighLevelControllerState;
   }

   public RobotController getController(FullHumanoidRobotModel fullRobotModel, double controlDT, double gravity, YoDouble yoTime,
                                        YoGraphicsListRegistry yoGraphicsListRegistry, DRCRobotSensorInformation sensorInformation,
                                        ForceSensorDataHolderReadOnly forceSensorDataHolder, CenterOfMassDataHolderReadOnly centerOfMassDataHolder,
                                        ContactSensorHolder contactSensorHolder, CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                        InverseDynamicsJoint... jointsToIgnore)
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
      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = walkingControllerParameters.getJointPrivilegedConfigurationParameters();
      double omega0 = walkingControllerParameters.getOmega0();
      controllerToolbox = new HighLevelHumanoidControllerToolbox(fullRobotModel, referenceFrames, footSwitches, centerOfMassDataHolder, wristForceSensors,
                                                                 yoTime, gravityZ, omega0, feet, controlDT, updatables, contactablePlaneBodies, yoGraphicsListRegistry, jointsToIgnore);
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
      double defaultInitialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime, defaultSwingTime, defaultInitialTransferTime, feet, statusMessageOutputManager, yoTime, yoGraphicsListRegistry, registry);
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);

      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);

      ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver = controllerToolbox.getReferenceFrameHashCodeResolver();
      FrameMessageCommandConverter commandConversionHelper = new FrameMessageCommandConverter(referenceFrameHashCodeResolver);
      commandInputManager.registerConversionHelper(commandConversionHelper);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the WholeBodyInverseDynamicsControlCore ////////////////////////////////////////////
      InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, rootJoint, jointsToOptimizeFor, centerOfMassFrame,
                                                                            momentumOptimizationSettings, yoGraphicsListRegistry, registry);
      toolbox.setJointPrivilegedConfigurationParameters(jointPrivilegedConfigurationParameters);
      if (setupInverseDynamicsSolver)
         toolbox.setupForInverseDynamicsSolver(contactablePlaneBodies);
      if (setupInverseKinematicsSolver)
         toolbox.setupForInverseKinematicsSolver();
      if (setupVirtualModelControlSolver)
      {
         RigidBody[] controlledBodies = {fullRobotModel.getPelvis(), fullRobotModel.getFoot(RobotSide.LEFT), fullRobotModel.getFoot(RobotSide.RIGHT)};
         toolbox.setupForVirtualModelControlSolver(fullRobotModel.getPelvis(), controlledBodies, contactablePlaneBodies);
      }
      FeedbackControlCommandList template = managerFactory.createFeedbackControlTemplate();
      WholeBodyControllerCore controllerCore = new WholeBodyControllerCore(toolbox, template, registry);
      ControllerCoreOutputReadOnly controllerCoreOutput = controllerCore.getOutputForHighLevelController();

      NewHumanoidHighLevelControllerManager highLevelHumanoidControllerManager = new NewHumanoidHighLevelControllerManager(commandInputManager, statusMessageOutputManager,
                                                                                                                           controllerCore, initialControllerState,
                                                                                                                           highLevelControllerParameters, walkingControllerParameters,
                                                                                                                           icpPlannerParameters, requestedHighLevelControllerState,
                                                                                                                           controllerFactoriesMap, stateTransitionFactories, managerFactory,
                                                                                                                           controllerToolbox, centerOfPressureDataHolderForEstimator,
                                                                                                                           controllerCoreOutput);
      highLevelHumanoidControllerManager.addYoVariableRegistry(registry);
      return highLevelHumanoidControllerManager;
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

   public void createControllerNetworkSubscriber(PeriodicThreadScheduler scheduler, PacketCommunicator packetCommunicator)
   {
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(commandInputManager, statusMessageOutputManager, scheduler,
                                                                                                packetCommunicator);
      closeableAndDisposableRegistry.registerCloseableAndDisposable(controllerNetworkSubscriber);
   }
}
