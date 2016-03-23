package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
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
import us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired.UserDesiredFootstepDataMessageGenerator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.DoNothingBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.KinematicsBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchAndContactSensorFusedFootSwitch;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureListener;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerStateChangedListener;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicThreadScheduler;

public class MomentumBasedControllerFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private boolean createComponentBasedFootstepDataMessageGenerator = false;
   private boolean useHeadingAndVelocityScript = true;
   private boolean createControllerNetworkSubscriber = false;
   private boolean createQueuedControllerCommandGenerator = false;
   private boolean createUserDesiredControllerCommandGenerator = true;

   private ConcurrentLinkedQueue<Command<?, ?>> controllerCommands;

   private final WalkingControllerParameters walkingControllerParameters;
   private final ArmControllerParameters armControllerParameters;
   private final CapturePointPlannerParameters capturePointPlannerParameters;

   private final HighLevelState initialBehavior;

   private MomentumBasedController momentumBasedController = null;

   private boolean isListeningToHighLevelStatePackets = true;
   private HighLevelHumanoidControllerManager highLevelHumanoidControllerManager = null;
   private final ArrayList<HighLevelBehavior> highLevelBehaviors = new ArrayList<>();

   private VariousWalkingManagers variousWalkingManagers;

   private ArrayList<HighLevelBehaviorFactory> highLevelBehaviorFactories = new ArrayList<>();

   private final SideDependentList<String> footSensorNames;
   private final SideDependentList<String> footContactSensorNames;
   private final SideDependentList<String> wristSensorNames;
   private final ContactableBodiesFactory contactableBodiesFactory;

   private WalkingHighLevelHumanoidController walkingBehavior;

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();

   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListenersToAttach = new ArrayList<>();
   private final ArrayList<ControllerFailureListener> controllerFailureListenersToAttach = new ArrayList<>();

   private HumanoidGlobalDataProducer globalDataProducer;
   private CloseableAndDisposableRegistry closeableAndDisposableRegistry;
   private PeriodicThreadScheduler scheduler;

   public MomentumBasedControllerFactory(ContactableBodiesFactory contactableBodiesFactory, SideDependentList<String> footForceSensorNames,
         SideDependentList<String> footContactSensorNames, SideDependentList<String> wristSensorNames, WalkingControllerParameters walkingControllerParameters,
         ArmControllerParameters armControllerParameters, CapturePointPlannerParameters capturePointPlannerParameters, HighLevelState initialBehavior)
   {
      this.footSensorNames = footForceSensorNames;
      this.footContactSensorNames = footContactSensorNames;
      this.wristSensorNames = wristSensorNames;
      this.contactableBodiesFactory = contactableBodiesFactory;
      this.initialBehavior = initialBehavior;

      this.walkingControllerParameters = walkingControllerParameters;
      this.armControllerParameters = armControllerParameters;
      this.capturePointPlannerParameters = capturePointPlannerParameters;

      commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
      statusOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());
   }

   public void addUpdatable(Updatable updatable)
   {
      this.updatables.add(updatable);
   }

   public HighLevelHumanoidControllerManager getHighLevelHumanoidControllerManager()
   {
      return highLevelHumanoidControllerManager;
   }

   public void setInverseDynamicsCalculatorListener(InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener)
   {
      momentumBasedController.setInverseDynamicsCalculatorListener(inverseDynamicsCalculatorListener);
   }

   public void createControllerNetworkSubscriber(PeriodicThreadScheduler scheduler)
   {
      this.scheduler = scheduler;
      if (globalDataProducer != null)
         new ControllerNetworkSubscriber(commandInputManager, statusOutputManager, closeableAndDisposableRegistry, scheduler, globalDataProducer);
      else
         createControllerNetworkSubscriber = true;
   }

   private ComponentBasedFootstepDataMessageGenerator footstepGenerator;

   public void createComponentBasedFootstepDataMessageGenerator(boolean useHeadingAndVelocityScript)
   {
      if (footstepGenerator != null)
         return;

      if (momentumBasedController != null)
      {
         SideDependentList<ContactableFoot> contactableFeet = momentumBasedController.getContactableFeet();
         CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
         double controlDT = momentumBasedController.getControlDT();
         ComponentBasedFootstepDataMessageGenerator footstepGenerator = new ComponentBasedFootstepDataMessageGenerator(commandInputManager, statusOutputManager,
               walkingControllerParameters, referenceFrames, contactableFeet, controlDT, useHeadingAndVelocityScript, registry);
         momentumBasedController.addUpdatables(footstepGenerator.getModulesToUpdate());
      }
      else
      {
         createComponentBasedFootstepDataMessageGenerator = true;
         this.useHeadingAndVelocityScript = useHeadingAndVelocityScript;
      }
   }

   private QueuedControllerCommandGenerator queuedControllerCommandGenerator;

   public void createQueuedControllerCommandGenerator(ConcurrentLinkedQueue<Command<?, ?>> controllerCommands)
   {
      if (queuedControllerCommandGenerator != null)
         return;

      if (momentumBasedController != null)
      {
         System.out.println("In createdQueuedControllerCommandGenerator");

         SideDependentList<ContactableFoot> contactableFeet = momentumBasedController.getContactableFeet();
         CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
         double controlDT = momentumBasedController.getControlDT();
         queuedControllerCommandGenerator = new QueuedControllerCommandGenerator(controllerCommands, commandInputManager, statusOutputManager,
               walkingControllerParameters, referenceFrames, contactableFeet, controlDT, useHeadingAndVelocityScript, registry);

         momentumBasedController.addUpdatables(queuedControllerCommandGenerator.getModulesToUpdate());
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
      if (userDesiredControllerCommandGenerators != null) return;

      if (momentumBasedController != null)
      {
         double defaultTrajectoryTime = 1.0;
         SideDependentList<ContactableFoot> contactableFeet = momentumBasedController.getContactableFeet();
         userDesiredControllerCommandGenerators = new UserDesiredControllerCommandGenerators(commandInputManager, momentumBasedController.getFullRobotModel(),
               contactableFeet, walkingControllerParameters, defaultTrajectoryTime, registry);

         momentumBasedController.addUpdatables(userDesiredControllerCommandGenerators.getUpdatables());
      }
      else
      {
         createUserDesiredControllerCommandGenerator = true;
      }
   }

   public RobotController getController(FullHumanoidRobotModel fullRobotModel, CommonHumanoidReferenceFrames referenceFrames, double controlDT, double gravity,
         DoubleYoVariable yoTime, YoGraphicsListRegistry yoGraphicsListRegistry, CloseableAndDisposableRegistry closeableAndDisposableRegistry,
         TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian, ForceSensorDataHolderReadOnly forceSensorDataHolder,
         ContactSensorHolder contactSensorHolder, CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator, HumanoidGlobalDataProducer dataProducer,
         InverseDynamicsJoint... jointsToIgnore)
   {
      this.closeableAndDisposableRegistry = closeableAndDisposableRegistry;
      this.globalDataProducer = dataProducer;
      SideDependentList<ContactableFoot> feet = contactableBodiesFactory.createFootContactableBodies(fullRobotModel, referenceFrames);

      double gravityZ = Math.abs(gravity);
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      double totalRobotWeight = totalMass * gravityZ;

      SideDependentList<FootSwitchInterface> footSwitches = createFootSwitches(feet, forceSensorDataHolder, contactSensorHolder, totalRobotWeight,
            yoGraphicsListRegistry, registry);
      SideDependentList<ForceSensorDataReadOnly> wristForceSensors = createWristForceSensors(forceSensorDataHolder);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the different ContactablePlaneBodies ///////////////////////////////////////////////

      RigidBody rootBody = fullRobotModel.getRootJoint().getSuccessor();
      SideDependentList<ContactablePlaneBody> handContactableBodies = contactableBodiesFactory.createHandContactableBodies(rootBody);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the MomentumBasedController ////////////////////////////////////////////////////////
      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();
      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      momentumBasedController = new MomentumBasedController(fullRobotModel, geometricJacobianHolder, centerOfMassJacobian, referenceFrames, footSwitches, wristForceSensors,
            yoTime, gravityZ, twistCalculator, feet, handContactableBodies, controlDT, updatables, armControllerParameters, walkingControllerParameters,
            yoGraphicsListRegistry, jointsToIgnore);
      momentumBasedController.attachControllerStateChangedListeners(controllerStateChangedListenersToAttach);
      attachControllerFailureListeners(controllerFailureListenersToAttach);
      if (createComponentBasedFootstepDataMessageGenerator)
         createComponentBasedFootstepDataMessageGenerator(useHeadingAndVelocityScript);
      if (createUserDesiredControllerCommandGenerator)
      if (createQueuedControllerCommandGenerator)
         createQueuedControllerCommandGenerator(controllerCommands);
      if (createUserDesiredControllerCommandGenerator)
         createUserDesiredControllerCommandGenerator();
      if (createControllerNetworkSubscriber)
         createControllerNetworkSubscriber(scheduler);

      variousWalkingManagers = new VariousWalkingManagers(statusOutputManager, momentumBasedController, walkingControllerParameters,
            capturePointPlannerParameters, armControllerParameters, registry);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the WholeBodyInverseDynamicsControlCore ////////////////////////////////////////////
      InverseDynamicsJoint[] jointsToOptimizeFor = MomentumBasedController.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);
      List<? extends ContactablePlaneBody> contactablePlaneBodies = momentumBasedController.getContactablePlaneBodyList();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(fullRobotModel, jointsToOptimizeFor, momentumOptimizationSettings, referenceFrames,
            controlDT, gravityZ, geometricJacobianHolder, twistCalculator, contactablePlaneBodies, yoGraphicsListRegistry);
      FeedbackControlCommandList template = variousWalkingManagers.createFeedbackControlTemplate();
      WholeBodyControllerCore controllerCore = new WholeBodyControllerCore(toolbox, template, registry);
      ControllerCoreOutputReadOnly controllerCoreOutput = controllerCore.getOutputForHighLevelController();

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the WalkingHighLevelHumanoidController /////////////////////////////////////////////

      walkingBehavior = new WalkingHighLevelHumanoidController(commandInputManager, statusOutputManager, variousWalkingManagers, walkingControllerParameters,
            momentumBasedController);
      highLevelBehaviors.add(walkingBehavior);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the DoNothingController ////////////////////////////////////////////////////////////
      // Useful as a transition state on the real robot
      DoNothingBehavior doNothingBehavior = new DoNothingBehavior(momentumBasedController);
      highLevelBehaviors.add(doNothingBehavior);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the HighLevelHumanoidControllerManager /////////////////////////////////////////////
      // This is the "highest level" controller that enables switching between
      // the different controllers (walking, multi-contact, driving, etc.)
      highLevelHumanoidControllerManager = new HighLevelHumanoidControllerManager(commandInputManager, controllerCore, initialBehavior, highLevelBehaviors,
            momentumBasedController, centerOfPressureDataHolderForEstimator, controllerCoreOutput, dataProducer);
      highLevelHumanoidControllerManager.setFallbackControllerForFailure(HighLevelState.DO_NOTHING_BEHAVIOR);
      highLevelHumanoidControllerManager.addYoVariableRegistry(registry);
      highLevelHumanoidControllerManager.setListenToHighLevelStatePackets(isListeningToHighLevelStatePackets);

      createRegisteredControllers();

      return highLevelHumanoidControllerManager;
   }

   private SideDependentList<FootSwitchInterface> createFootSwitches(SideDependentList<? extends ContactablePlaneBody> bipedFeet,
         ForceSensorDataHolderReadOnly forceSensorDataHolder, ContactSensorHolder contactSensorHolder, double totalRobotWeight,
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
                  bipedFeet.get(robotSide), yoGraphicsListRegistry, contactThresholdForce, registry);
            wrenchBasedFootSwitch.setSecondContactThresholdForce(walkingControllerParameters.getSecondContactThresholdForceIgnoringCoP());
            footSwitch = wrenchBasedFootSwitch;
            break;

         case WrenchAndContactSensorFused:
            footSwitch = new WrenchAndContactSensorFusedFootSwitch(footName, footForceSensor,
                  contactSensorHolder.getByName(footContactSensorNames.get(robotSide)), footSwitchCoPThresholdFraction, totalRobotWeight,
                  bipedFeet.get(robotSide), yoGraphicsListRegistry, contactThresholdForce, registry);
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
      for (int i = 0; i < highLevelBehaviorFactories.size(); i++)
      {
         HighLevelBehaviorFactory highLevelBehaviorFactory = highLevelBehaviorFactories.get(i);
         HighLevelBehavior highLevelBehavior = highLevelBehaviorFactory.createHighLevelBehavior(variousWalkingManagers, momentumBasedController);
         boolean transitionRequested = highLevelBehaviorFactory.isTransitionToBehaviorRequested();
         highLevelHumanoidControllerManager.addHighLevelBehavior(highLevelBehavior, transitionRequested);
      }
   }

   public void reinitializeWalking(boolean keepPosition)
   {
      highLevelHumanoidControllerManager.requestHighLevelState(HighLevelState.WALKING);
      if (keepPosition)
      {
         if (walkingBehavior != null)
         {
            walkingBehavior.initializeDesiredHeightToCurrent();
            walkingBehavior.reinitializePelvisOrientation(false);
         }

         if (variousWalkingManagers != null)
         {
            variousWalkingManagers.getManipulationControlModule().initializeDesiredToCurrent();
            variousWalkingManagers.getPelvisOrientationManager().setToHoldCurrentInWorldFrame();
         }
      }
   }

   public void reinitializePositionControl()
   {
      highLevelHumanoidControllerManager.requestHighLevelState(HighLevelState.JOINT_POSITION_CONTROL);
   }

   public void setListenToHighLevelStatePackets(boolean isListening)
   {
      if (highLevelHumanoidControllerManager != null)
         highLevelHumanoidControllerManager.setListenToHighLevelStatePackets(isListening);
      else
         isListeningToHighLevelStatePackets = isListening;
   }

   public void addHighLevelBehaviorFactory(HighLevelBehaviorFactory highLevelBehaviorFactory)
   {
      if (momentumBasedController == null)
      {
         highLevelBehaviorFactories.add(highLevelBehaviorFactory);
      }
      else
      {
         HighLevelBehavior highLevelBehavior = highLevelBehaviorFactory.createHighLevelBehavior(variousWalkingManagers, momentumBasedController);
         boolean transitionToBehaviorRequested = highLevelBehaviorFactory.isTransitionToBehaviorRequested();
         highLevelHumanoidControllerManager.addHighLevelBehavior(highLevelBehavior, transitionToBehaviorRequested);
      }
   }

   public void attachControllerFailureListeners(List<ControllerFailureListener> listeners)
   {
      for (int i = 0; i < listeners.size(); i++)
         attachControllerFailureListener(listeners.get(i));
   }

   public void attachControllerFailureListener(ControllerFailureListener listener)
   {
      if (momentumBasedController != null)
         momentumBasedController.attachControllerFailureListener(listener);
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
      if (momentumBasedController != null)
         momentumBasedController.attachControllerStateChangedListener(listener);
      else
         controllerStateChangedListenersToAttach.add(listener);
   }

   public void attachRobotMotionStatusChangedListener(RobotMotionStatusChangedListener listener)
   {
      momentumBasedController.attachRobotMotionStatusChangedListener(listener);
   }

   public void setFallbackControllerForFailure(HighLevelState fallbackController)
   {
      highLevelHumanoidControllerManager.setFallbackControllerForFailure(fallbackController);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }
}
