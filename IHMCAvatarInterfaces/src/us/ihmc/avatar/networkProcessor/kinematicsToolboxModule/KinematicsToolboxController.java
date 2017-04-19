package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.communication.kinematicsToolboxAPI.KinematicsToolboxInputCommand;
import us.ihmc.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class KinematicsToolboxController extends ToolboxController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double updateDT = 1.0e-3;
   private static final int numberOfTicksToSendSolution = 10;
   private static final String centerOfMassName = "CenterOfMass";

   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final RigidBody elevator;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final TwistCalculator twistCalculator;

   private final YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("genericGains", registry);
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
   private final WholeBodyControllerCore controllerCore;
   private final FeedbackControllerDataReadOnly feedbackControllerDataHolder;

   private final KinematicsToolboxOutputStatus inverseKinematicsSolution;

   private final FloatingInverseDynamicsJoint desiredRootJoint;
   private final OneDoFJoint[] oneDoFJoints;

   private final SideDependentList<BooleanYoVariable> isFootInSupport = new SideDependentList<>();
   private final SideDependentList<YoFramePoseUsingQuaternions> initialFootPoses = new SideDependentList<>();
   private final YoFramePoint initialCenterOfMassPosition = new YoFramePoint("initialCenterOfMass", worldFrame, registry);

   private final BooleanYoVariable holdSupportFootPose = new BooleanYoVariable("holdSupportFootPose", registry);
   private final BooleanYoVariable holdCenterOfMassXYPosition = new BooleanYoVariable("holdCenterOfMassXYPosition", registry);

   private final DoubleYoVariable privilegedWeight = new DoubleYoVariable("privilegedWeight", registry);
   private final DoubleYoVariable privilegedConfigurationGain = new DoubleYoVariable("privilegedConfigurationGain", registry);
   private final DoubleYoVariable privilegedMaxVelocity = new DoubleYoVariable("privilegedMaxVelocity", registry);
   private final DoubleYoVariable footWeight = new DoubleYoVariable("footWeight", registry);
   private final DoubleYoVariable momentumWeight = new DoubleYoVariable("momentumWeight", registry);

   private final CommandInputManager commandInputManager;
   private final DoubleYoVariable solutionQuality = new DoubleYoVariable("solutionQuality", registry);
   private int tickCount = 0;

   private final EnumMap<LegJointName, DoubleYoVariable> legJointLimitReductionFactors = new EnumMap<>(LegJointName.class);

   private final List<RigidBody> listOfControllableRigidBodies = new ArrayList<>();

   private final AtomicReference<PrivilegedConfigurationCommand> privilegedConfigurationCommandReference = new AtomicReference<PrivilegedConfigurationCommand>(null);

   private final Map<RigidBody, YoGraphicCoordinateSystem> desiredCoodinateSystems = new HashMap<>();
   private final Map<RigidBody, YoGraphicCoordinateSystem> currentCoodinateSystems = new HashMap<>();

   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<>(null);
   private final AtomicReference<CapturabilityBasedStatus> latestCapturabilityBasedStatusReference = new AtomicReference<>(null);

   private final Map<String, FeedbackControlCommand<?>> activeFeedbackCommands = new HashMap<>();

   public KinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                      FullHumanoidRobotModel desiredFullRobotModel, YoGraphicsListRegistry yoGraphicsListRegistry,
                                      YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.commandInputManager = commandInputManager;

      this.desiredFullRobotModel = desiredFullRobotModel;

      referenceFrames = new HumanoidReferenceFrames(desiredFullRobotModel);
      elevator = desiredFullRobotModel.getElevator();
      desiredRootJoint = desiredFullRobotModel.getRootJoint();
      twistCalculator = new TwistCalculator(worldFrame, elevator);

      populateJointLimitReductionFactors();
      populateListOfControllableRigidBodies();

      controllerCore = createControllerCore();
      feedbackControllerDataHolder = controllerCore.getWholeBodyFeedbackControllerDataHolder();

      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(desiredFullRobotModel);
      inverseKinematicsSolution = new KinematicsToolboxOutputStatus(oneDoFJoints);
      inverseKinematicsSolution.setDestination(-1);
      gains.setProportionalGain(800.0);

      footWeight.set(200.0);
      momentumWeight.set(1.0);
      privilegedWeight.set(1.0);
      privilegedConfigurationGain.set(50.0);
      privilegedMaxVelocity.set(Double.POSITIVE_INFINITY);

      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         isFootInSupport.put(robotSide, new BooleanYoVariable("is" + side + "FootInSupport", registry));
         initialFootPoses.put(robotSide, new YoFramePoseUsingQuaternions(sidePrefix + "FootInitial", worldFrame, registry));
      }

      setupVisualization(yoGraphicsListRegistry);
   }

   public void setupVisualization(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);
         RigidBody hand = desiredFullRobotModel.getHand(robotSide);
         AppearanceDefinition desiredAppearance = YoAppearance.Red();
         AppearanceDefinition currentAppearance = YoAppearance.Blue();

         desiredCoodinateSystems.put(foot, createCoodinateSystem(foot, Type.DESIRED, desiredAppearance));
         desiredCoodinateSystems.put(hand, createCoodinateSystem(hand, Type.DESIRED, desiredAppearance));
         currentCoodinateSystems.put(foot, createCoodinateSystem(foot, Type.CURRENT, currentAppearance));
         currentCoodinateSystems.put(hand, createCoodinateSystem(hand, Type.CURRENT, currentAppearance));
      }

      desiredCoodinateSystems.forEach((k, v) -> yoGraphicsListRegistry.registerYoGraphic("CoordinateSystems", v));
      currentCoodinateSystems.forEach((k, v) -> yoGraphicsListRegistry.registerYoGraphic("CoordinateSystems", v));
   }

   public WholeBodyControllerCore createControllerCore()
   {
      InverseDynamicsJoint[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(desiredFullRobotModel);
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(updateDT, 0.0, desiredRootJoint, controlledJoints, centerOfMassFrame,
                                                                            twistCalculator, null, null, registry);
      toolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());
      toolbox.setupForInverseKinematicsSolver();
      FeedbackControlCommandList controllerCoreTemplate = createControllerCoreTemplate();
      controllerCoreTemplate.addCommand(new CenterOfMassFeedbackControlCommand());
      return new WholeBodyControllerCore(toolbox, controllerCoreTemplate, registry);
   }

   public void populateJointLimitReductionFactors()
   {
      DoubleYoVariable hipReductionFactor = new DoubleYoVariable("hipLimitReductionFactor", registry);
      DoubleYoVariable kneeReductionFactor = new DoubleYoVariable("kneeLimitReductionFactor", registry);
      DoubleYoVariable ankleReductionFactor = new DoubleYoVariable("ankleLimitReductionFactor", registry);
      hipReductionFactor.set(0.05);

      legJointLimitReductionFactors.put(LegJointName.HIP_PITCH, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_ROLL, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_YAW, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.KNEE_PITCH, kneeReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_PITCH, ankleReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_ROLL, ankleReductionFactor);
   }

   private void populateListOfControllableRigidBodies()
   {
      listOfControllableRigidBodies.add(desiredFullRobotModel.getChest());
      listOfControllableRigidBodies.add(desiredFullRobotModel.getPelvis());

      for (RobotSide robotSide : RobotSide.values)
      {
         listOfControllableRigidBodies.add(desiredFullRobotModel.getHand(robotSide));
         listOfControllableRigidBodies.add(desiredFullRobotModel.getFoot(robotSide));
      }
   }

   private FeedbackControlCommandList createControllerCoreTemplate()
   {
      FeedbackControlCommandList template = new FeedbackControlCommandList();
      listOfControllableRigidBodies.stream().map(this::createFeedbackControlCommand).forEach(template::addCommand);
      return template;
   }

   private SpatialFeedbackControlCommand createFeedbackControlCommand(RigidBody endEffector)
   {
      SpatialFeedbackControlCommand command = new SpatialFeedbackControlCommand();
      command.set(elevator, endEffector);
      return command;
   }

   private YoGraphicCoordinateSystem createCoodinateSystem(RigidBody endEffector, Type type, AppearanceDefinition appearanceDefinition)
   {
      String namePrefix = endEffector.getName() + type.getName();
      return new YoGraphicCoordinateSystem(namePrefix, "", registry, 0.2, appearanceDefinition);
   }

   @Override
   protected boolean initialize()
   {
      activeFeedbackCommands.clear();

      RobotConfigurationData robotConfigurationData = latestRobotConfigurationDataReference.get();

      if (robotConfigurationData == null)
         return false;

      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationData, desiredRootJoint, oneDoFJoints);

      CapturabilityBasedStatus capturabilityBasedStatus = latestCapturabilityBasedStatusReference.get();

      if (capturabilityBasedStatus == null)
      {
         for (RobotSide robotSide : RobotSide.values)
            isFootInSupport.get(robotSide).set(true);
      }
      else
      {
         for (RobotSide robotside : RobotSide.values)
            isFootInSupport.get(robotside).set(capturabilityBasedStatus.isSupportFoot(robotside));
      }

      updateTools();

      initialCenterOfMassPosition.setFromReferenceFrame(referenceFrames.getCenterOfMassFrame());

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);
         initialFootPoses.get(robotSide).setFromReferenceFrame(foot.getBodyFixedFrame());
      }

      holdSupportFootPose.set(true);
      holdCenterOfMassXYPosition.set(true);

      PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_CURRENT);
      privilegedConfigurationCommand.setDefaultWeight(privilegedWeight.getDoubleValue());
      privilegedConfigurationCommand.setDefaultConfigurationGain(privilegedConfigurationGain.getDoubleValue());
      privilegedConfigurationCommand.setDefaultMaxVelocity(privilegedMaxVelocity.getDoubleValue());
      privilegedConfigurationCommandReference.set(privilegedConfigurationCommand);

      return true;
   }

   private final IntegerYoVariable numberOfActiveCommands = new IntegerYoVariable("numberOfActiveCommands", registry);

   @Override
   protected void updateInternal()
   {
      updateTools();

      controllerCoreCommand.clear();

      controllerCoreCommand.addFeedbackControlCommand(createHoldCenterOfMassXYCommand());
      controllerCoreCommand.addFeedbackControlCommand(createHoldSupportFootCommands());
      FeedbackControlCommandList userCommands = consumeCommands();
      numberOfActiveCommands.set(userCommands.getNumberOfCommands());
      controllerCoreCommand.addFeedbackControlCommand(userCommands);

      controllerCoreCommand.addInverseKinematicsCommand(createJointLimitReductionCommand());
      controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommandReference.getAndSet(null));

      solutionQuality.set(KinematicsToolboxHelper.calculateSolutionQuality(controllerCoreCommand.getFeedbackControlCommandList(), feedbackControllerDataHolder));

      controllerCore.reset();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      KinematicsToolboxHelper.setRobotStateFromControllerCoreOutput(controllerCore.getControllerCoreOutput(), desiredRootJoint, oneDoFJoints);
      updateVisualization();

      inverseKinematicsSolution.setDesiredJointState(desiredRootJoint, oneDoFJoints);
      inverseKinematicsSolution.setSolutionQuality(solutionQuality.getDoubleValue());

      tickCount++;
      if (tickCount == numberOfTicksToSendSolution)
      {
         reportMessage(inverseKinematicsSolution);
         tickCount = 0;
      }
   }

   public void updateTools()
   {
      desiredFullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      twistCalculator.compute();
   }

   private FeedbackControlCommandList consumeCommands()
   {

      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxInputCommand.class))
      {
         KinematicsToolboxInputCommand newestCommand = commandInputManager.pollNewestCommand(KinematicsToolboxInputCommand.class);

         holdCenterOfMassXYPosition.set(newestCommand.holdCurrentCenterOfMassXYPosition());
         holdSupportFootPose.set(newestCommand.holdSupporFootPositions());

         if (newestCommand.hasCenterOfMassTaskBeenSet())
            activeFeedbackCommands.put(centerOfMassName, KinematicsToolboxHelper.consumeCenterOfMassCommand(newestCommand.getCenterOfMassTask(), gains));

         for (int i = 0; i < newestCommand.getNumberOfEndEffectorTasks(); i++)
         {
            SpatialFeedbackControlCommand rigidBodyCommand = KinematicsToolboxHelper.consumeRigidBodyCommand(newestCommand.getEndEffectorTask(i), elevator, gains);
            activeFeedbackCommands.put(rigidBodyCommand.getEndEffector().getName(), rigidBodyCommand);
         }
      }

      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxCenterOfMassCommand.class))
      {
         KinematicsToolboxCenterOfMassCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxCenterOfMassCommand.class);
         activeFeedbackCommands.put(centerOfMassName, KinematicsToolboxHelper.consumeCenterOfMassCommand(command, gains));
      }

      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxRigidBodyCommand.class))
      {
         List<KinematicsToolboxRigidBodyCommand> commands = commandInputManager.pollNewCommands(KinematicsToolboxRigidBodyCommand.class);
         for (int i = 0; i < commands.size(); i++)
         {
            SpatialFeedbackControlCommand rigidBodyCommand = KinematicsToolboxHelper.consumeRigidBodyCommand(commands.get(i), elevator, gains);
            activeFeedbackCommands.put(rigidBodyCommand.getEndEffector().getName(), rigidBodyCommand);
         }
      }

      FeedbackControlCommandList inputs = new FeedbackControlCommandList();
      activeFeedbackCommands.values().forEach(inputs::addCommand);
      return inputs;
   }

   private FeedbackControlCommand<?> createHoldSupportFootCommands()
   {
      if (!holdSupportFootPose.getBooleanValue())
         return null;

      FeedbackControlCommandList inputs = new FeedbackControlCommandList();

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isFootInSupport.get(robotSide).getBooleanValue())
            continue;

         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);
         FramePose poseToHold = new FramePose();
         initialFootPoses.get(robotSide).getFramePoseIncludingFrame(poseToHold);

         SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
         feedbackControlCommand.set(elevator, foot);
         feedbackControlCommand.setGains((SE3PIDGainsInterface) gains);
         feedbackControlCommand.setWeightForSolver(footWeight.getDoubleValue());
         feedbackControlCommand.set(poseToHold);
         inputs.addCommand(feedbackControlCommand);
      }
      return inputs;
   }

   private FeedbackControlCommand<?> createHoldCenterOfMassXYCommand()
   {
      if (!holdCenterOfMassXYPosition.getBooleanValue())
         return null;

      FramePoint positionToHold = new FramePoint();
      initialCenterOfMassPosition.getFrameTupleIncludingFrame(positionToHold);

      CenterOfMassFeedbackControlCommand feedbackControlCommand = new CenterOfMassFeedbackControlCommand();
      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightForSolver(momentumWeight.getDoubleValue());
      feedbackControlCommand.setSelectionMatrixForLinearXYControl();
      feedbackControlCommand.set(positionToHold);
      return feedbackControlCommand;
   }

   private JointLimitReductionCommand createJointLimitReductionCommand()
   {
      JointLimitReductionCommand jointLimitReductionCommand = new JointLimitReductionCommand();
      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : desiredFullRobotModel.getRobotSpecificJointNames().getLegJointNames())
         {
            OneDoFJoint joint = desiredFullRobotModel.getLegJoint(robotSide, legJointName);
            double reductionFactor = legJointLimitReductionFactors.get(legJointName).getDoubleValue();
            jointLimitReductionCommand.addReductionFactor(joint, reductionFactor);
         }
      }
      return jointLimitReductionCommand;
   }

   private void updateVisualization()
   {
      boolean hasData;
      FramePoint position = new FramePoint();
      FrameOrientation orientation = new FrameOrientation();

      for (RigidBody endEffector : desiredCoodinateSystems.keySet())
      {
         YoGraphicCoordinateSystem coordinateSystem = desiredCoodinateSystems.get(endEffector);
         hasData = feedbackControllerDataHolder.getPositionData(endEffector, position, Type.DESIRED);
         if (!hasData)
            coordinateSystem.hide();
         else
            coordinateSystem.setPosition(position);

         hasData = feedbackControllerDataHolder.getOrientationData(endEffector, orientation, Type.DESIRED);
         if (!hasData)
            coordinateSystem.hide();
         else
            coordinateSystem.setOrientation(orientation);
      }

      for (RigidBody endEffector : currentCoodinateSystems.keySet())
      {
         YoGraphicCoordinateSystem coordinateSystem = currentCoodinateSystems.get(endEffector);
         hasData = feedbackControllerDataHolder.getPositionData(endEffector, position, Type.CURRENT);
         if (!hasData)
            coordinateSystem.hide();
         else
            coordinateSystem.setPosition(position);

         hasData = feedbackControllerDataHolder.getOrientationData(endEffector, orientation, Type.CURRENT);
         if (!hasData)
            coordinateSystem.hide();
         else
            coordinateSystem.setOrientation(orientation);
      }
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      latestRobotConfigurationDataReference.set(newConfigurationData);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      latestCapturabilityBasedStatusReference.set(newStatus);
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }

   @Override
   protected boolean isDone()
   {
      // This toolbox should run until if falls asleep.
      return false;
   }

   public KinematicsToolboxOutputStatus getSolution()
   {
      return inverseKinematicsSolution;
   }
}