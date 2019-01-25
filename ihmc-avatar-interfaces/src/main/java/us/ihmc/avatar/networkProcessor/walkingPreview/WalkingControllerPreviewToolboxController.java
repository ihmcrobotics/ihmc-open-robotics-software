package us.ihmc.avatar.networkProcessor.walkingPreview;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.walkingPreviewToolboxAPI.WalkingControllerPreviewInputCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.tools.taskExecutor.TaskExecutor;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkingControllerPreviewToolboxController extends ToolboxController
{
   private final double gravityZ = 9.81;
   private final YoDouble previewTime;
   private final double integrationDT = 0.001;

   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private final FullHumanoidRobotModel fullRobotModel;
   private final CommonHumanoidReferenceFrames referenceFrames;
   // TODO Think how the foot switch should be updated/implemented. We can use the parameter WalkingControllerParameters.finishSingleSupportWhenICPPlannerIsDone to finish swing but is that enough?
   private final SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<>();

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WholeBodyControllerCore controllerCore;
   private final WalkingHighLevelHumanoidController walkingController;

   private final CommandInputManager toolboxInputManager;

   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<>(null);
   private final CommandInputManager walkingInputManager = new CommandInputManager("walking_preview_internal",
                                                                                   ControllerAPIDefinition.getControllerSupportedCommands());
   private final StatusMessageOutputManager walkingOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());

   private final JointAccelerationIntegrationCommand jointAccelerationIntegrationCommand = new JointAccelerationIntegrationCommand();

   private final TaskExecutor taskExecutor = new TaskExecutor();

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final List<KinematicsToolboxOutputStatus> previewFrames = new ArrayList<>();

   public WalkingControllerPreviewToolboxController(DRCRobotModel robotModel, CommandInputManager toolboxInputManager,
                                                    StatusMessageOutputManager statusOutputManager, YoGraphicsListRegistry yoGraphicsListRegistry,
                                                    YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.toolboxInputManager = toolboxInputManager;
      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      previewTime = new YoDouble("timeInPreview", registry);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();

      controllerToolbox = createHighLevelControllerToolbox(robotModel, yoGraphicsListRegistry);
      rootJoint = fullRobotModel.getRootJoint();
      controlledOneDoFJoints = controllerToolbox.getControlledOneDoFJoints();

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      HighLevelControlManagerFactory managerFactory = new HighLevelControlManagerFactory(statusOutputManager, registry);
      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);

      walkingController = new WalkingHighLevelHumanoidController(walkingInputManager, walkingOutputManager, managerFactory, walkingControllerParameters,
                                                                 controllerToolbox);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      WholeBodyControlCoreToolbox controlCoreToolbox = createControllerCoretoolbox(walkingControllerParameters, yoGraphicsListRegistry);

      FeedbackControlCommandList feedbackControlTemplate = managerFactory.createFeedbackControlTemplate();
      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(controlledOneDoFJoints);

      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, feedbackControlTemplate, jointDesiredOutputList, registry);
      walkingController.setControllerCoreOutput(controllerCore.getOutputForHighLevelController());

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      for (OneDoFJointBasics joint : controlledOneDoFJoints)
      {
         int jointIndex = jointAccelerationIntegrationCommand.getNumberOfJointsToComputeDesiredPositionFor();
         jointAccelerationIntegrationCommand.addJointToComputeDesiredPositionFor(joint);
         jointAccelerationIntegrationCommand.setBreakFrequencies(jointIndex, 0.0, 0.0); // Setup for pure integration without leak.
         jointAccelerationIntegrationCommand.setJointMaxima(jointIndex, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      }

      registry.addChild(walkingController.getYoVariableRegistry());
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
         footSwitches.put(robotSide, new SettableFootSwitch(feet.get(robotSide), robotMass, 2, registry));
      }

      JointBasics[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());

      return new HighLevelHumanoidControllerToolbox(fullRobotModel, referenceFrames, footSwitches, null, null, previewTime, gravityZ, omega0, feet,
                                                    integrationDT, Collections.emptyList(), allContactableBodies, yoGraphicsListRegistry, jointsToIgnore);
   }

   private WholeBodyControlCoreToolbox createControllerCoretoolbox(WalkingControllerParameters walkingControllerParameters,
                                                                   YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double robotMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      JointBasics[] controlledJoints = controllerToolbox.getControlledJoints();
      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = walkingControllerParameters.getJointPrivilegedConfigurationParameters();
      FeedbackControllerSettings feedbackControllerSettings = walkingControllerParameters.getFeedbackControllerSettings();

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(integrationDT, robotMass, fullRobotModel.getRootJoint(),
                                                                                       controlledJoints, controllerToolbox.getCenterOfMassFrame(),
                                                                                       momentumOptimizationSettings, yoGraphicsListRegistry, registry);

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
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i), additionaContactNames.get(i),
                                                            additionalContactTransforms.get(i));

      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);

      return contactableBodiesFactory;
   }

   @Override
   public boolean initialize()
   {
      isDone.set(false);
      previewTime.set(0.0);
      previewFrames.clear();

      RobotConfigurationData robotConfigurationData = latestRobotConfigurationDataReference.get();
      if (robotConfigurationData == null)
         return false;

      // Initializes this desired robot to the most recent robot configuration data received from the walking controller.
      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationData, rootJoint, controlledOneDoFJoints);

      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      walkingController.initialize();

      taskExecutor.clear();
      taskExecutor.submit(new WalkingPreviewResetTask(fullRobotModel, controllerToolbox.getFootContactStates(), walkingInputManager, controllerToolbox));

      return true;
   }

   @Override
   public void updateInternal() throws Exception
   {
      if (toolboxInputManager.isNewCommandAvailable(WalkingControllerPreviewInputCommand.class))
      {
         initialize();
         WalkingControllerPreviewInputCommand command = toolboxInputManager.pollNewestCommand(WalkingControllerPreviewInputCommand.class);
         FootstepDataListCommand foostepCommand = command.getFoostepCommand();
         taskExecutor.submit(new FootstepListPreviewTask(foostepCommand, walkingInputManager, walkingOutputManager, controllerToolbox.getFootContactStates()));
      }
      else
      {
         previewTime.add(integrationDT);
         fullRobotModel.updateFrames();
         referenceFrames.updateFrames();
      }

      taskExecutor.doControl();

      walkingController.doAction();
      ControllerCoreCommand controllerCoreCommand = walkingController.getControllerCoreCommand();
      controllerCoreCommand.addInverseDynamicsCommand(jointAccelerationIntegrationCommand);
      controllerCoreCommand.addInverseDynamicsCommand(((WalkingPreviewTask) taskExecutor.getCurrentTask()).getOutput());
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      KinematicsToolboxHelper.setRobotStateFromControllerCoreOutput(controllerCore.getControllerCoreOutput(), rootJoint, controlledOneDoFJoints);

      if (!(taskExecutor.getCurrentTask() instanceof WalkingPreviewResetTask)) // Skip the frames that are to reset the controller.
         previewFrames.add(MessageTools.createKinematicsToolboxOutputStatus(rootJoint, controlledOneDoFJoints));

      isDone.set(taskExecutor.isDone());

      if (isDone.getValue())
         reportMessage(MessageTools.createWalkingControllerPreviewOutputMessage(integrationDT, previewFrames));
   }

   @Override
   public boolean isDone()
   {
      return isDone.getValue();
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      latestRobotConfigurationDataReference.set(newConfigurationData);
   }
}
