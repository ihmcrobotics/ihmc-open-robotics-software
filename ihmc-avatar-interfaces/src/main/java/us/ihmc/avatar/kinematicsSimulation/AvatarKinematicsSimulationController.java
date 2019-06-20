package us.ihmc.avatar.kinematicsSimulation;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WalkingControllerPreviewOutputMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModule;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisHeightControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.stateMachine.core.StateMachineClock;
import us.ihmc.robotics.taskExecutor.StateExecutor;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class AvatarKinematicsSimulationController
{
   private final double gravityZ = 9.81;
   private final double integrationDT;
   private final YoDouble previewTime;
   private final YoBoolean hasControllerFailed;
   private final YoBoolean isInitialized;
   private final YoBoolean isDone;

   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] allOneDoFJointsExcludingHands;
   private final FullHumanoidRobotModel fullRobotModel;
   private final YoVariableRegistry registry;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final SideDependentList<SettableFootSwitch> footSwitches = new SideDependentList<>();

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WholeBodyControllerCore controllerCore;
   private final LinearMomentumRateControlModule linearMomentumRateControlModule;
   private final HighLevelControlManagerFactory managerFactory;
   private final WalkingHighLevelHumanoidController walkingController;

   private final CommandInputManager kinematicsSimulationInputManager;

   // is this for the initial condition?
   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<>(null);
   private final CommandInputManager walkingInputManager
         = new CommandInputManager("walking_preview_internal", ControllerAPIDefinition.getControllerSupportedCommands());
   private final StatusMessageOutputManager walkingOutputManager
         = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

   private final StateExecutor taskExecutor = new StateExecutor(StateMachineClock.dummyClock()); // should be dummy?
   private final List<KinematicsToolboxOutputStatus> previewFrames = new ArrayList<>();
   private final MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator();
   private final List<Updatable> updatables = new ArrayList<>();

   public AvatarKinematicsSimulationController(DRCRobotModel robotModel,
                                               double integrationDT,
                                               YoGraphicsListRegistry yoGraphicsListRegistry,
                                               YoVariableRegistry registry)
   {
      this.integrationDT = integrationDT;
      this.registry = registry;

      kinematicsSimulationInputManager = new CommandInputManager("ik_simulation", ControllerAPIDefinition.getControllerSupportedCommands());
      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      previewTime = new YoDouble("timeInPreview", registry);
      isInitialized = new YoBoolean("isInitialized", registry);
      isDone = new YoBoolean("isDone", registry);
      hasControllerFailed = new YoBoolean("hasControllerFailed", registry);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();

      // Create registries to match controller so the XML gets loaded properly.
      YoVariableRegistry drcControllerThreadRegistry = new YoVariableRegistry("DRCControllerThread");
      YoVariableRegistry drcMomentumBasedControllerRegistry = new YoVariableRegistry("DRCMomentumBasedController");
      YoVariableRegistry humanoidHighLevelControllerManagerRegistry = new YoVariableRegistry("HumanoidHighLevelControllerManager");
      YoVariableRegistry managerParentRegistry = new YoVariableRegistry("HighLevelHumanoidControllerFactory");
      YoVariableRegistry walkingParentRegistry = new YoVariableRegistry("WalkingControllerState");
      registry.addChild(drcControllerThreadRegistry);
      drcControllerThreadRegistry.addChild(drcMomentumBasedControllerRegistry);
      drcMomentumBasedControllerRegistry.addChild(humanoidHighLevelControllerManagerRegistry);
      humanoidHighLevelControllerManagerRegistry.addChild(walkingParentRegistry);
      humanoidHighLevelControllerManagerRegistry.addChild(managerParentRegistry);

      controllerToolbox = createHighLevelControllerToolbox(robotModel, yoGraphicsListRegistry);
      controllerToolbox.attachControllerFailureListener(fallingDirection -> hasControllerFailed.set(true));
      humanoidHighLevelControllerManagerRegistry.addChild(controllerToolbox.getYoVariableRegistry());
      setupWalkingMessageHandler(walkingControllerParameters, yoGraphicsListRegistry);
      rootJoint = fullRobotModel.getRootJoint();
      allOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      managerFactory = new HighLevelControlManagerFactory(managerParentRegistry);
      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);

      walkingController = new WalkingHighLevelHumanoidController(walkingInputManager,
                                                                 walkingOutputManager,
                                                                 managerFactory,
                                                                 walkingControllerParameters,
                                                                 controllerToolbox);
      walkingParentRegistry.addChild(walkingController.getYoVariableRegistry());

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      WholeBodyControlCoreToolbox controlCoreToolbox = createControllerCoretoolbox(walkingControllerParameters, yoGraphicsListRegistry);

      FeedbackControlCommandList feedbackControlTemplate = managerFactory.createFeedbackControlTemplate();
      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(controllerToolbox.getControlledOneDoFJoints());

      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, feedbackControlTemplate, jointDesiredOutputList, walkingParentRegistry);
      walkingController.setControllerCoreOutput(controllerCore.getOutputForHighLevelController());

      double controlDT = controllerToolbox.getControlDT();
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      YoDouble yoTime = controllerToolbox.getYoTime();
      SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();
      linearMomentumRateControlModule = new LinearMomentumRateControlModule(referenceFrames,
                                                                            contactableFeet,
                                                                            elevator,
                                                                            walkingControllerParameters,
                                                                            yoTime,
                                                                            gravityZ,
                                                                            controlDT,
                                                                            walkingParentRegistry,
                                                                            yoGraphicsListRegistry);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


      ParameterLoaderHelper.loadParameters(this, robotModel, drcControllerThreadRegistry);

      YoVariable<?> defaultHeight = registry.getVariable(PelvisHeightControlState.class.getSimpleName(),
                                                         PelvisHeightControlState.class.getSimpleName() + "DefaultHeight");
      if (Double.isNaN(defaultHeight.getValueAsDouble()))
      {
         throw new RuntimeException("Need to load a default height.");
      }
   }

   private HighLevelHumanoidControllerToolbox createHighLevelControllerToolbox(DRCRobotModel robotModel, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double omega0 = robotModel.getWalkingControllerParameters().getOmega0();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = createContactableBodiesFactory(robotModel);
      SideDependentList<ContactableFoot> feet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      List<ContactablePlaneBody> additionalContacts = contactableBodiesFactory.createAdditionalContactPoints();
      contactableBodiesFactory.disposeFactory();

      List<ContactablePlaneBody> allContactableBodies = new ArrayList<>(additionalContacts);
      allContactableBodies.addAll(feet.values());

      double robotMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      for (RobotSide robotSide : RobotSide.values)
      {
         SettableFootSwitch footSwitch = new SettableFootSwitch(feet.get(robotSide), robotMass, 2, registry);
         footSwitch.setFootContactState(true);
         footSwitches.put(robotSide, footSwitch);
      }

      JointBasics[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());

      return new HighLevelHumanoidControllerToolbox(fullRobotModel,
                                                    referenceFrames,
                                                    footSwitches,
                                                    null,
                                                    previewTime,
                                                    gravityZ,
                                                    omega0,
                                                    feet,
                                                    integrationDT,
                                                    Collections.emptyList(),
                                                    allContactableBodies,
                                                    yoGraphicsListRegistry,
                                                    jointsToIgnore);
   }

   private void setupWalkingMessageHandler(WalkingControllerParameters walkingControllerParameters, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      double defaultInitialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double defaultFinalTransferTime = walkingControllerParameters.getDefaultFinalTransferTime();
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime,
                                                                              defaultSwingTime,
                                                                              defaultInitialTransferTime,
                                                                              defaultFinalTransferTime,
                                                                              controllerToolbox.getContactableFeet(),
                                                                              walkingOutputManager,
                                                                              previewTime,
                                                                              yoGraphicsListRegistry,
                                                                              controllerToolbox.getYoVariableRegistry());
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);
   }

   private WholeBodyControlCoreToolbox createControllerCoretoolbox(WalkingControllerParameters walkingControllerParameters,
                                                                   YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      JointBasics[] controlledJoints = controllerToolbox.getControlledJoints();
      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = walkingControllerParameters.getJointPrivilegedConfigurationParameters();
      FeedbackControllerSettings feedbackControllerSettings = walkingControllerParameters.getFeedbackControllerSettings();

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(integrationDT,
                                                                                       gravityZ,
                                                                                       fullRobotModel.getRootJoint(),
                                                                                       controlledJoints,
                                                                                       controllerToolbox.getCenterOfMassFrame(),
                                                                                       momentumOptimizationSettings,
                                                                                       yoGraphicsListRegistry,
                                                                                       registry);

      controlCoreToolbox.setJointPrivilegedConfigurationParameters(jointPrivilegedConfigurationParameters);
      controlCoreToolbox.setFeedbackControllerSettings(feedbackControllerSettings);
      controlCoreToolbox.setupForInverseDynamicsSolver(controllerToolbox.getContactablePlaneBodies());

      return controlCoreToolbox;
   }

   private ContactableBodiesFactory<RobotSide> createContactableBodiesFactory(DRCRobotModel robotModel)
   {
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionaContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i),
                                                            additionaContactNames.get(i),
                                                            additionalContactTransforms.get(i));

      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);

      return contactableBodiesFactory;
   }

   public boolean initialize()
   {
      isDone.set(false);
      isInitialized.set(false);
      hasControllerFailed.set(false);
      return latestRobotConfigurationDataReference.get() != null;
   }

   private void initializeInternal()
   {
      LogTools.info("Initializing");
      isDone.set(false);
      previewTime.set(0.0);
      previewFrames.clear();

      RobotConfigurationData robotConfigurationData = latestRobotConfigurationDataReference.get();

      // Initializes this desired robot to the most recent robot configuration data received from the walking controller.
      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationData,
                                                                      rootJoint,
                                                                      FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel));

      for (JointBasics joint : fullRobotModel.getElevator().childrenSubtreeIterable())
      {
         joint.setJointTwistToZero();
         joint.setJointAccelerationToZero();
      }

      referenceFrames.updateFrames();
      controllerCore.initialize();
      controllerToolbox.update();
      walkingController.initialize();

      taskExecutor.clear();
      taskExecutor.submit(new KinematicsWalkingHaltTask(walkingController, controllerToolbox.getFootContactStates()));
      isInitialized.set(true);
   }

   public void updateInternal()
   {
      if (isDone())
      {
         for (JointBasics joint : fullRobotModel.getElevator().childrenSubtreeIterable())
         {
            joint.setJointAccelerationToZero();
            joint.setJointTwistToZero();
         }
         return;
      }

      if (kinematicsSimulationInputManager.isNewCommandAvailable(FootstepDataListCommand.class))
      {
         initializeInternal();
         FootstepDataListCommand foostepCommand = kinematicsSimulationInputManager.pollNewestCommand(FootstepDataListCommand.class);
         taskExecutor.submit(new KinematicsWalkingFootstepSequenceTask(fullRobotModel.getRootJoint(),
                                                                       foostepCommand,
                                                                       walkingInputManager,
                                                                       walkingOutputManager,
                                                                       controllerToolbox.getFootContactStates(),
                                                                       managerFactory.getOrCreateBalanceManager(),
                                                                       footSwitches));
      }
      else
      {
         previewTime.add(integrationDT);
         fullRobotModel.updateFrames();
         referenceFrames.updateFrames();
         controllerToolbox.update();
      }

      if (!isInitialized.getValue())
         return;

      taskExecutor.doControl();

      walkingController.doAction();

      linearMomentumRateControlModule.setInputFromWalkingStateMachine(walkingController.getLinearMomentumRateControlModuleInput());
      if (!linearMomentumRateControlModule.computeControllerCoreCommands())
      {
         controllerToolbox.reportControllerFailureToListeners(new FrameVector2D());
      }
      walkingController.setLinearMomentumRateControlModuleOutput(linearMomentumRateControlModule.getOutputForWalkingStateMachine());

      ControllerCoreCommand controllerCoreCommand = walkingController.getControllerCoreCommand();
      controllerCoreCommand.addInverseDynamicsCommand(linearMomentumRateControlModule.getMomentumRateCommand());
      if (!taskExecutor.isDone())
         controllerCoreCommand.addInverseDynamicsCommand(((KinematicsWalkingTask) taskExecutor.getCurrentTask()).getOutput());

      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      linearMomentumRateControlModule.setInputFromControllerCore(controllerCore.getControllerCoreOutput());
      linearMomentumRateControlModule.computeAchievedCMP();

      rootJoint.setJointAcceleration(0, controllerCore.getOutputForRootJoint().getDesiredAcceleration());
      JointDesiredOutputListReadOnly jointDesiredOutputList = controllerCore.getOutputForLowLevelController();

      for (OneDoFJointBasics joint : controllerToolbox.getControlledOneDoFJoints())
      {
         JointDesiredOutputReadOnly jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         joint.setQdd(jointDesiredOutput.getDesiredAcceleration());
      }

      integrator.setIntegrationDT(integrationDT);
      integrator.doubleIntegrateFromAcceleration(Arrays.asList(controllerToolbox.getControlledJoints()));

      if (!(taskExecutor.getCurrentTask() instanceof KinematicsWalkingHaltTask))
         previewFrames.add(MessageTools.createKinematicsToolboxOutputStatus(rootJoint, allOneDoFJointsExcludingHands));

      isDone.set(taskExecutor.isDone() || hasControllerFailed.getValue());

      if (isDone())
      {
         LogTools.info("Preview is done, packing and sending result, number of frames: " + previewFrames.size());
         if (hasControllerFailed.getValue())
            LogTools.info("Controller has failed.");
         WalkingControllerPreviewOutputMessage output = MessageTools.createWalkingControllerPreviewOutputMessage(integrationDT, previewFrames);
         LogTools.info(output.toString());
         taskExecutor.clear();
      }

      updatables.forEach(updatable -> updatable.update(previewTime.getValue()));
   }

   public CommandInputManager getInputManager()
   {
      return kinematicsSimulationInputManager;
   }

   public StatusMessageOutputManager getWalkingOutputManager()
   {
      return walkingOutputManager;
   }

   public void addUpdatable(Updatable updatable)
   {
      this.updatables.add(updatable);
   }

   public boolean isDone()
   {
      return isDone.getValue();
   }

   public boolean isWalkingControllerResetDone()
   {
      return isInitialized.getValue() && !(taskExecutor.getCurrentTask() instanceof KinematicsWalkingHaltTask);
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      latestRobotConfigurationDataReference.set(newConfigurationData);
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public double getIntegrationDT()
   {
      return integrationDT;
   }
}
