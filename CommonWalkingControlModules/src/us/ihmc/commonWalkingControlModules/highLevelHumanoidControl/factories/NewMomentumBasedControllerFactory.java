package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
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
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.*;
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
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.sensors.*;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

public class NewMomentumBasedControllerFactory extends AbstractMomentumBasedControllerFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private boolean createComponentBasedFootstepDataMessageGenerator = false;
   private boolean useHeadingAndVelocityScript = true;
   private HeightMap heightMapForFootstepZ = null;
   private boolean createQueuedControllerCommandGenerator = false;
   private boolean createUserDesiredControllerCommandGenerator = true;

   private ConcurrentLinkedQueue<Command<?, ?>> controllerCommands;
   private final YoEnum<NewHighLevelControllerStates> requestedHighLevelControllerState = new YoEnum<NewHighLevelControllerStates>("requestedHighLevelControllerState", registry,
                                                                                                                                   NewHighLevelControllerStates.class, true);
   private final WalkingControllerParameters walkingControllerParameters;
   private final ICPTrajectoryPlannerParameters capturePointPlannerParameters;

   private final StandPrepParameters standPrepSetpoints;
   private final PositionControlParameters positionControlParameters;
   private final NewHighLevelControllerStates initialControllerState;
   private final NewHighLevelControllerStates fallbackControllerState;

   private HighLevelHumanoidControllerToolbox controllerToolbox = null;

   private boolean isListeningToHighLevelStatePackets = true;
   private NewHumanoidHighLevelControllerManager highLevelControllerStateMachine = null;
   private final ArrayList<NewHighLevelControllerState> highLevelControllerStates = new ArrayList<>();
   private final HashMap<NewHighLevelControllerState, ArrayList<StateTransition<NewHighLevelControllerStates>>> highLevelControllerTransitions = new HashMap<>();

   private final HighLevelControlManagerFactory managerFactory;

   private ArrayList<NewHighLevelControllerFactory> highLevelControllerFactories = new ArrayList<>();

   private final SideDependentList<String> footSensorNames;
   private final SideDependentList<String> footContactSensorNames;
   private final SideDependentList<String> wristSensorNames;
   private final ContactableBodiesFactory contactableBodiesFactory;

   private WalkingHighLevelHumanoidController walkingState;

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();

   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListenersToAttach = new ArrayList<>();
   private final ArrayList<ControllerFailureListener> controllerFailureListenersToAttach = new ArrayList<>();

   private final CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   private HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters;

   private boolean setupInverseDynamicsSolver = true;
   private boolean setupInverseKinematicsSolver = false;
   private boolean setupVirtualModelControlSolver = false;

   public NewMomentumBasedControllerFactory(ContactableBodiesFactory contactableBodiesFactory, SideDependentList<String> footForceSensorNames,
                                            SideDependentList<String> footContactSensorNames, SideDependentList<String> wristSensorNames,
                                            WalkingControllerParameters walkingControllerParameters, ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters,
                                            StandPrepParameters standPrepSetpoints, PositionControlParameters positionControlParameters,
                                            NewHighLevelControllerStates initialControllerState, NewHighLevelControllerStates fallbackControllerState)
   {
      this.footSensorNames = footForceSensorNames;
      this.footContactSensorNames = footContactSensorNames;
      this.wristSensorNames = wristSensorNames;
      this.contactableBodiesFactory = contactableBodiesFactory;
      this.standPrepSetpoints = standPrepSetpoints;
      this.positionControlParameters = positionControlParameters;
      this.initialControllerState = initialControllerState;
      this.fallbackControllerState = fallbackControllerState;

      this.walkingControllerParameters = walkingControllerParameters;
      this.capturePointPlannerParameters = capturePointPlannerParameters;

      commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
      statusOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

      managerFactory = new HighLevelControlManagerFactory(statusOutputManager, registry);
      managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);
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
    * <p>            requestedState.set(null);
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

   public void setHeadingAndVelocityEvaluationScriptParameters(HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      headingAndVelocityEvaluationScriptParameters = walkingScriptParameters;
   }

   public void addUpdatable(Updatable updatable)
   {
      this.updatables.add(updatable);
   }

   public void setInverseDynamicsCalculatorListener(InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener)
   {
      controllerToolbox.setInverseDynamicsCalculatorListener(inverseDynamicsCalculatorListener);
   }

   public void createControllerNetworkSubscriber(PeriodicThreadScheduler scheduler, PacketCommunicator packetCommunicator)
   {
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(commandInputManager, statusOutputManager, scheduler,
                                                                                                packetCommunicator);
      closeableAndDisposableRegistry.registerCloseableAndDisposable(controllerNetworkSubscriber);
   }

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
      if (controllerToolbox != null)
      {
         SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();
         CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
         double controlDT = controllerToolbox.getControlDT();
         ComponentBasedFootstepDataMessageGenerator footstepGenerator = new ComponentBasedFootstepDataMessageGenerator(commandInputManager, statusOutputManager,
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
         queuedControllerCommandGenerator = new QueuedControllerCommandGenerator(controllerCommands, commandInputManager, statusOutputManager,
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

   @Override
   public void closeAndDispose()
   {
      closeableAndDisposableRegistry.closeAndDispose();
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
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime, defaultSwingTime, defaultInitialTransferTime, feet, statusOutputManager, yoTime, yoGraphicsListRegistry, registry);
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);

      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      
      ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver = controllerToolbox.getReferenceFrameHashCodeResolver();
      FrameMessageCommandConverter commandConversionHelper = new FrameMessageCommandConverter(referenceFrameHashCodeResolver);
      commandInputManager.registerConversionHelper(commandConversionHelper);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the WalkingController //////////////////////////////////////////////////////////////
      walkingState = new WalkingHighLevelHumanoidController(commandInputManager, statusOutputManager, managerFactory,
                                                                                                    walkingControllerParameters, capturePointPlannerParameters,
                                                                                                    controllerToolbox);

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

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the DoNothingController ////////////////////////////////////////////////////////////
      // Useful as a transition state on the real robot
      NewDoNothingControllerState doNothingState = createDoNothingControllerState(controllerToolbox);
      highLevelControllerStates.add(doNothingState);
      ArrayList<StateTransition<NewHighLevelControllerStates>> doNothingTransitions = new ArrayList<>();
      doNothingTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, NewHighLevelControllerStates.STAND_PREP_STATE));
      doNothingTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, NewHighLevelControllerStates.CALIBRATION));
      highLevelControllerTransitions.put(doNothingState, doNothingTransitions);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the WalkingController //////////////////////////////////////////////////////////////
      NewWalkingControllerState walkingControllerState = createWalkingControllerState(walkingState, controllerToolbox, controllerCore);
      highLevelControllerStates.add(walkingControllerState);
      ArrayList<StateTransition<NewHighLevelControllerStates>> walkingTransitions = new ArrayList<>();
      walkingTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, NewHighLevelControllerStates.FREEZE_STATE));
      walkingTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, fallbackControllerState));
      highLevelControllerTransitions.put(walkingControllerState, walkingTransitions);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the NewStandPrepControllerState ////////////////////////////////////////////////////////////
      NewStandPrepControllerState standPrepControllerState = createStandPrepControllerState(controllerToolbox, standPrepSetpoints, positionControlParameters);
      highLevelControllerStates.add(standPrepControllerState);
      ArrayList<StateTransition<NewHighLevelControllerStates>> standPrepTransitions = new ArrayList<>();
      standPrepTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, NewHighLevelControllerStates.FREEZE_STATE));
      standPrepTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, fallbackControllerState));
      standPrepTransitions.add(StateMachineTools.buildFinishedStateTransition(standPrepControllerState, NewHighLevelControllerStates.STAND_READY));
      highLevelControllerTransitions.put(standPrepControllerState, standPrepTransitions);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the StandReadyController ///////////////////////////////////////////////////////////
      NewStandReadyControllerState standReadyControllerState = createStandReadyControllerState(controllerToolbox, standPrepSetpoints, positionControlParameters);
      highLevelControllerStates.add(standReadyControllerState);
      ArrayList<StateTransition<NewHighLevelControllerStates>> standReadyTransitions = new ArrayList<>();
      standReadyTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, NewHighLevelControllerStates.FREEZE_STATE));
      standReadyTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, fallbackControllerState));
      standReadyTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, NewHighLevelControllerStates.STAND_TRANSITION_STATE));
      StateTransition<NewHighLevelControllerStates> feetLoadedTransition = new StateTransition<>(NewHighLevelControllerStates.STAND_TRANSITION_STATE,
           new FeetLoadedTransition(forceSensorDataHolder, sensorInformation.getFeetForceSensorNames(), controllerToolbox.getControlDT(), registry));
      standReadyTransitions.add(feetLoadedTransition);
      highLevelControllerTransitions.put(standReadyControllerState, standReadyTransitions);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the StandTransitionController //////////////////////////////////////////////////////
      NewStandTransitionControllerState standTransitionControllerState = createStandTransitionControllerState(standReadyControllerState, walkingControllerState, controllerToolbox);
      highLevelControllerStates.add(standTransitionControllerState);
      ArrayList<StateTransition<NewHighLevelControllerStates>> standTransitionTransitions = new ArrayList<>();
      standTransitionTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, NewHighLevelControllerStates.FREEZE_STATE));
      standTransitionTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, fallbackControllerState));
      standTransitionTransitions.add(StateMachineTools.buildFinishedStateTransition(standTransitionControllerState, NewHighLevelControllerStates.WALKING_STATE));
      highLevelControllerTransitions.put(standTransitionControllerState, standTransitionTransitions);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the FreezeController ///////////////////////////////////////////////////////////
      NewFreezeControllerState freezeControllerState = createFreezeControllerState(controllerToolbox, standPrepSetpoints, positionControlParameters);
      highLevelControllerStates.add(freezeControllerState);
      ArrayList<StateTransition<NewHighLevelControllerStates>> freezeTransitions = new ArrayList<>();
      freezeTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, fallbackControllerState));
      freezeTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, NewHighLevelControllerStates.DO_NOTHING_STATE));
      freezeTransitions.add(StateMachineTools.buildRequestableStateTransition(requestedHighLevelControllerState, NewHighLevelControllerStates.STAND_READY));
      highLevelControllerTransitions.put(freezeControllerState, freezeTransitions);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the CalibrationController //////////////////////////////////////////////////////////
      // todo

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the DiagnosticsController //////////////////////////////////////////////////////////
      // todo

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the HighLevelHumanoidControllerManager /////////////////////////////////////////////
      // This is the "highest level" controller that enables switching between
      // the different controllers (walking, multi-contact, driving, etc.)
      highLevelControllerStateMachine = new NewHumanoidHighLevelControllerManager(commandInputManager, statusOutputManager, controllerCore, initialControllerState,
                                                                                  requestedHighLevelControllerState, highLevelControllerStates,
                                                                                  highLevelControllerTransitions, controllerToolbox,
                                                                                  centerOfPressureDataHolderForEstimator, controllerCoreOutput);
      highLevelControllerStateMachine.addYoVariableRegistry(registry);
      highLevelControllerStateMachine.setListenToHighLevelStatePackets(isListeningToHighLevelStatePackets);

      createRegisteredControllers();

      return highLevelControllerStateMachine;
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

   private void createRegisteredControllers()
   {
      for (int i = 0; i < highLevelControllerFactories.size(); i++)
      {
         NewHighLevelControllerFactory highLevelBehaviorFactory = highLevelControllerFactories.get(i);
         NewHighLevelControllerState highLevelControllerState = highLevelBehaviorFactory.createHighLevelController(managerFactory, controllerToolbox);
         boolean transitionRequested = highLevelBehaviorFactory.isTransitionToBehaviorRequested();
         highLevelControllerStateMachine.addHighLevelController(highLevelControllerState, transitionRequested);
      }
   }

   public void reinitializeWalking(boolean keepPosition)
   {
      highLevelControllerStateMachine.requestHighLevelControllerState(NewHighLevelControllerStates.WALKING_STATE);
      if (keepPosition)
      {
         if (walkingState != null)
         {
            walkingState.initializeDesiredHeightToCurrent();
            walkingState.reinitializePelvisOrientation(false);
         }

         if (managerFactory != null)
         {
            managerFactory.getOrCreatePelvisOrientationManager().setToHoldCurrentInWorldFrame();
         }
      }
   }

   public void setListenToHighLevelStatePackets(boolean isListening)
   {
      if (highLevelControllerStateMachine != null)
         highLevelControllerStateMachine.setListenToHighLevelStatePackets(isListening);
      else
         isListeningToHighLevelStatePackets = isListening;
   }

   public void addHighLevelBehaviorFactory(NewHighLevelControllerFactory highLevelControllerFactory)
   {
      if (controllerToolbox == null)
      {
         highLevelControllerFactories.add(highLevelControllerFactory);
      }
      else
      {
         NewHighLevelControllerState highLevelControllerState = highLevelControllerFactory.createHighLevelController(managerFactory, controllerToolbox);
         boolean transitionToBehaviorRequested = highLevelControllerFactory.isTransitionToBehaviorRequested();
         highLevelControllerStateMachine.addHighLevelController(highLevelControllerState, transitionToBehaviorRequested);
      }
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

   public void setFallbackControllerForFailure(NewHighLevelControllerStates fallbackController)
   {
      highLevelControllerStateMachine.setFallbackControllerForFailure(fallbackController);
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   public StatusMessageOutputManager getStatusOutputManager()
   {
      return statusOutputManager;
   }

   public HighLevelHumanoidControllerToolbox getHighLevelHumanoidControllerToolbox()
   {
      if (controllerToolbox == null)
         throw new RuntimeException(HighLevelHumanoidControllerToolbox.class.getSimpleName() + " has not been created yet.");

      return controllerToolbox;
   }

   /**
    * Warms up the walking controller by running it a number of iterations
    * 
    * @param iterations
    */
   public void warmupWalkingController(int iterations)
   {
      highLevelControllerStateMachine.warmup(iterations, walkingState);
   }

   public HighLevelState getCurrentHighLevelState()
   {
      return null;
   }

   public NewDoNothingControllerState createDoNothingControllerState(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      return new NewDoNothingControllerState(controllerToolbox);
   }

   public NewStandPrepControllerState createStandPrepControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepParameters standPrepSetpoints,
                                                                     PositionControlParameters positionControlParameters)
   {
      return new NewStandPrepControllerState(controllerToolbox, standPrepSetpoints, positionControlParameters);
   }


   public NewStandReadyControllerState createStandReadyControllerState(HighLevelHumanoidControllerToolbox controllerToolbox,
                                                                       StandPrepParameters standPrepSetpoints, PositionControlParameters positionControlParameters)
   {
      return new NewStandReadyControllerState(controllerToolbox, standPrepSetpoints, positionControlParameters);
   }

   public NewFreezeControllerState createFreezeControllerState(HighLevelHumanoidControllerToolbox controllerToolbox,
                                                               StandPrepParameters standPrepSetpoints, PositionControlParameters positionControlParameters)
   {
      return new NewFreezeControllerState(controllerToolbox, standPrepSetpoints, positionControlParameters);
   }

   public NewStandTransitionControllerState createStandTransitionControllerState(NewStandReadyControllerState standReadyControllerState,
                                                                                 NewWalkingControllerState walkingControllerState,
                                                                                 HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      return new NewStandTransitionControllerState(standReadyControllerState, walkingControllerState, controllerToolbox);
   }

   public NewWalkingControllerState createWalkingControllerState(WalkingHighLevelHumanoidController walkingController,
                                                                 HighLevelHumanoidControllerToolbox controllerToolbox, WholeBodyControllerCore controllerCore)
   {
      return new NewWalkingControllerState(controllerToolbox, controllerCore, walkingController);
   }

}
