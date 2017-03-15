package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
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
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.DoNothingBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
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
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicThreadScheduler;

public class MomentumBasedControllerFactory implements CloseableAndDisposable
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

   private final WalkingControllerParameters walkingControllerParameters;
   private final CapturePointPlannerParameters capturePointPlannerParameters;

   private final HighLevelState initialBehavior;

   private HighLevelHumanoidControllerToolbox controllerToolbox = null;

   private boolean isListeningToHighLevelStatePackets = true;
   private HighLevelHumanoidControllerManager highLevelHumanoidControllerManager = null;
   private final ArrayList<HighLevelBehavior> highLevelBehaviors = new ArrayList<>();

   private final HighLevelControlManagerFactory managerFactory;

   private ArrayList<HighLevelBehaviorFactory> highLevelBehaviorFactories = new ArrayList<>();

   private final SideDependentList<String> footSensorNames;
   private final SideDependentList<String> footContactSensorNames;
   private final SideDependentList<String> wristSensorNames;
   private final ContactableBodiesFactory contactableBodiesFactory;

   private WalkingHighLevelHumanoidController walkingBehavior;

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();

   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListenersToAttach = new ArrayList<>();
   private final ArrayList<ControllerFailureListener> controllerFailureListenersToAttach = new ArrayList<>();

   private final CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   private HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters;

   public MomentumBasedControllerFactory(ContactableBodiesFactory contactableBodiesFactory, SideDependentList<String> footForceSensorNames,
         SideDependentList<String> footContactSensorNames, SideDependentList<String> wristSensorNames, WalkingControllerParameters walkingControllerParameters,
         CapturePointPlannerParameters capturePointPlannerParameters, HighLevelState initialBehavior)
   {
      this.footSensorNames = footForceSensorNames;
      this.footContactSensorNames = footContactSensorNames;
      this.wristSensorNames = wristSensorNames;
      this.contactableBodiesFactory = contactableBodiesFactory;
      this.initialBehavior = initialBehavior;

      this.walkingControllerParameters = walkingControllerParameters;
      this.capturePointPlannerParameters = capturePointPlannerParameters;

      commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
      statusOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

      managerFactory = new HighLevelControlManagerFactory(statusOutputManager, registry);
      managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);
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
         ComponentBasedFootstepDataMessageGenerator footstepGenerator = new ComponentBasedFootstepDataMessageGenerator(commandInputManager, statusOutputManager,
               walkingControllerParameters, headingAndVelocityEvaluationScriptParameters, referenceFrames, contactableFeet, controlDT, useHeadingAndVelocityScript, heightMapForFootstepZ, registry);
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
               walkingControllerParameters, referenceFrames, contactableFeet, controlDT, useHeadingAndVelocityScript, registry);

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
               controllerToolbox.getReferenceFrames(),
               contactableFeet, walkingControllerParameters, defaultTrajectoryTime, registry);
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

   public RobotController getController(FullHumanoidRobotModel fullRobotModel, double controlDT, double gravity, DoubleYoVariable yoTime,
         YoGraphicsListRegistry yoGraphicsListRegistry, ForceSensorDataHolderReadOnly forceSensorDataHolder, CenterOfMassDataHolderReadOnly centerOfMassDataHolder,
         ContactSensorHolder contactSensorHolder,
         CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator, InverseDynamicsJoint... jointsToIgnore)
   {
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());

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
      // Setup the HighLevelHumanoidControllerToolbox /////////////////////////////////////////////
      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();
      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = walkingControllerParameters.getJointPrivilegedConfigurationParameters();
      double omega0 = walkingControllerParameters.getOmega0();
      controllerToolbox = new HighLevelHumanoidControllerToolbox(fullRobotModel, geometricJacobianHolder, referenceFrames, footSwitches,
            centerOfMassDataHolder, wristForceSensors, yoTime,
            gravityZ, omega0, twistCalculator, feet, handContactableBodies, controlDT, updatables, yoGraphicsListRegistry, jointsToIgnore);
      controllerToolbox.attachControllerStateChangedListeners(controllerStateChangedListenersToAttach);
      attachControllerFailureListeners(controllerFailureListenersToAttach);
      if (createComponentBasedFootstepDataMessageGenerator)
         createComponentBasedFootstepDataMessageGenerator(useHeadingAndVelocityScript, heightMapForFootstepZ);
      if (createUserDesiredControllerCommandGenerator)
         if (createQueuedControllerCommandGenerator)
            createQueuedControllerCommandGenerator(controllerCommands);
      if (createUserDesiredControllerCommandGenerator)
         createUserDesiredControllerCommandGenerator();

      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the WalkingHighLevelHumanoidController /////////////////////////////////////////////
      walkingBehavior = new WalkingHighLevelHumanoidController(commandInputManager, statusOutputManager, managerFactory, walkingControllerParameters,
            capturePointPlannerParameters, controllerToolbox);
      highLevelBehaviors.add(walkingBehavior);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the DoNothingController ////////////////////////////////////////////////////////////
      // Useful as a transition state on the real robot
      DoNothingBehavior doNothingBehavior = new DoNothingBehavior(controllerToolbox);
      highLevelBehaviors.add(doNothingBehavior);

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the WholeBodyInverseDynamicsControlCore ////////////////////////////////////////////
      RigidBody[] controlledBodies = {fullRobotModel.getPelvis(), fullRobotModel.getFoot(RobotSide.LEFT), fullRobotModel.getFoot(RobotSide.RIGHT)};
      InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);
      List<? extends ContactablePlaneBody> contactablePlaneBodies = controllerToolbox.getContactablePlaneBodyList();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(fullRobotModel, controlledBodies, jointsToOptimizeFor, momentumOptimizationSettings,
            jointPrivilegedConfigurationParameters, referenceFrames, controlDT, gravityZ, geometricJacobianHolder, twistCalculator, contactablePlaneBodies,
            yoGraphicsListRegistry, registry);
      FeedbackControlCommandList template = managerFactory.createFeedbackControlTemplate();
      WholeBodyControllerCore controllerCore = new WholeBodyControllerCore(toolbox, template, registry);
      ControllerCoreOutputReadOnly controllerCoreOutput = controllerCore.getOutputForHighLevelController();

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the HighLevelHumanoidControllerManager /////////////////////////////////////////////
      // This is the "highest level" controller that enables switching between
      // the different controllers (walking, multi-contact, driving, etc.)
      highLevelHumanoidControllerManager = new HighLevelHumanoidControllerManager(commandInputManager, statusOutputManager, controllerCore, initialBehavior,
            highLevelBehaviors, controllerToolbox, centerOfPressureDataHolderForEstimator, controllerCoreOutput);
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
         HighLevelBehavior highLevelBehavior = highLevelBehaviorFactory.createHighLevelBehavior(managerFactory, controllerToolbox);
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

         if (managerFactory != null)
         {
            managerFactory.getOrCreatePelvisOrientationManager().setToHoldCurrentInWorldFrame();
         }
      }
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
      if (controllerToolbox == null)
      {
         highLevelBehaviorFactories.add(highLevelBehaviorFactory);
      }
      else
      {
         HighLevelBehavior highLevelBehavior = highLevelBehaviorFactory.createHighLevelBehavior(managerFactory, controllerToolbox);
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

   public void setFallbackControllerForFailure(HighLevelState fallbackController)
   {
      highLevelHumanoidControllerManager.setFallbackControllerForFailure(fallbackController);
   }

   public void setICPOptimizationControllerParameters(ICPOptimizationParameters icpOptimizationParameters)
   {
      managerFactory.setICPOptimizationParameters(icpOptimizationParameters);
   }

   public HighLevelState getCurrentHighLevelState()
   {
      return highLevelHumanoidControllerManager.getCurrentHighLevelState();
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
}
