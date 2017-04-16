package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TrackingWeightsCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
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

   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final TwistCalculator twistCalculator;

   private final YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("genericGains", registry);
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
   private final WholeBodyControllerCore controllerCore;
   private final FeedbackControllerDataReadOnly feedbackControllerDataHolder;

   private final KinematicsToolboxOutputStatus inverseKinematicsSolution;

   private final FloatingInverseDynamicsJoint desiredRootJoint;
   private final OneDoFJoint[] oneDoFJoints;

   // TODO Change everything to yovariables
   private final AtomicReference<FramePoint> desiredCenterOfMassReference = new AtomicReference<>(null);

   private final DoubleYoVariable privilegedWeight = new DoubleYoVariable("privilegedWeight", registry);
   private final DoubleYoVariable privilegedConfigurationGain = new DoubleYoVariable("privilegedConfigurationGain", registry);
   private final DoubleYoVariable privilegedMaxVelocity = new DoubleYoVariable("privilegedMaxVelocity", registry);
   private final DoubleYoVariable handWeight = new DoubleYoVariable("handWeight", registry);
   private final DoubleYoVariable footWeight = new DoubleYoVariable("footWeight", registry);
   private final DoubleYoVariable momentumWeight = new DoubleYoVariable("momentumWeight", registry);
   private final DoubleYoVariable chestWeight = new DoubleYoVariable("chestWeight", registry);
   private final DoubleYoVariable pelvisOrientationWeight = new DoubleYoVariable("pelvisOrientationWeight", registry);
   private final DoubleYoVariable pelvisHeightWeight = new DoubleYoVariable("pelvisHeightWeight", registry);

   private final CommandInputManager commandInputManager;
   private final DoubleYoVariable solutionQuality = new DoubleYoVariable("solutionQuality", registry);
   private int tickCount = 0;

   private final EnumMap<LegJointName, DoubleYoVariable> legJointLimitReductionFactors = new EnumMap<>(LegJointName.class);

   private final CenterOfMassFeedbackControlCommand centerOfMassFeedbackCommand = new CenterOfMassFeedbackControlCommand();
   private final OrientationFeedbackControlCommand chestFeedbackCommand = new OrientationFeedbackControlCommand();
   private final PointFeedbackControlCommand pelvisHeightFeedbackCommand = new PointFeedbackControlCommand();
   private final OrientationFeedbackControlCommand pelvisOrientationFeedbackCommand = new OrientationFeedbackControlCommand();
   private final SideDependentList<SpatialFeedbackControlCommand> handFeedbackCommands = new SideDependentList<>();
   private final SideDependentList<SpatialFeedbackControlCommand> footFeedbackCommands = new SideDependentList<>();

   private final Set<FeedbackControlCommand<?>> activeFeedbackControlCommands = new HashSet<>();

   private final AtomicReference<PrivilegedConfigurationCommand> privilegedConfigurationCommandReference = new AtomicReference<PrivilegedConfigurationCommand>(null);

   private final Map<RigidBody, YoGraphicCoordinateSystem> desiredCoodinateSystems = new HashMap<>();
   private final Map<RigidBody, YoGraphicCoordinateSystem> currentCoodinateSystems = new HashMap<>();

   public KinematicsToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                      FullHumanoidRobotModel desiredFullRobotModel, DRCRobotModel robotModel, YoGraphicsListRegistry yoGraphicsListRegistry,
                                      YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.commandInputManager = commandInputManager;

      this.desiredFullRobotModel = desiredFullRobotModel;
      InverseDynamicsJoint[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(desiredFullRobotModel);
      referenceFrames = new HumanoidReferenceFrames(desiredFullRobotModel);
      RigidBody elevator = desiredFullRobotModel.getElevator();
      twistCalculator = new TwistCalculator(worldFrame, elevator);

      MomentumOptimizationSettings momentumOptimizationSettings = robotModel.getWalkingControllerParameters().getMomentumOptimizationSettings();
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      desiredRootJoint = desiredFullRobotModel.getRootJoint();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(updateDT, 0.0, desiredRootJoint, controlledJoints, centerOfMassFrame,
                                                                            twistCalculator, momentumOptimizationSettings, yoGraphicsListRegistry, registry);
      toolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());
      toolbox.setupForInverseKinematicsSolver();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(desiredFullRobotModel);

      inverseKinematicsSolution = new KinematicsToolboxOutputStatus(oneDoFJoints);
      inverseKinematicsSolution.setDestination(-1);

      RigidBody chest = desiredFullRobotModel.getChest();
      RigidBody pelvis = desiredFullRobotModel.getPelvis();

      gains.setProportionalGain(800.0);

      FeedbackControlCommandList controllerCoreTemplate = new FeedbackControlCommandList();

      for (RobotSide robotSide : RobotSide.values)
      {
         SpatialFeedbackControlCommand handFeedbackCommand = new SpatialFeedbackControlCommand();
         handFeedbackCommand.set(elevator, desiredFullRobotModel.getHand(robotSide));
         //         handFeedbackCommand.setPrimaryBase(chest);
         handFeedbackCommands.put(robotSide, handFeedbackCommand);
         controllerCoreTemplate.addCommand(handFeedbackCommand);

         SpatialFeedbackControlCommand footFeedbackCommand = new SpatialFeedbackControlCommand();
         footFeedbackCommand.set(elevator, desiredFullRobotModel.getFoot(robotSide));
         //         footFeedbackCommand.setPrimaryBase(pelvis);
         footFeedbackCommands.put(robotSide, footFeedbackCommand);
         controllerCoreTemplate.addCommand(footFeedbackCommand);
      }

      chestFeedbackCommand.set(elevator, chest);
      //      chestFeedbackCommand.setPrimaryBase(pelvis);
      controllerCoreTemplate.addCommand(chestFeedbackCommand);
      pelvisHeightFeedbackCommand.set(elevator, pelvis);
      controllerCoreTemplate.addCommand(pelvisHeightFeedbackCommand);
      pelvisOrientationFeedbackCommand.set(elevator, pelvis);
      controllerCoreTemplate.addCommand(pelvisOrientationFeedbackCommand);
      controllerCoreTemplate.addCommand(centerOfMassFeedbackCommand);

      controllerCore = new WholeBodyControllerCore(toolbox, controllerCoreTemplate, registry);
      feedbackControllerDataHolder = controllerCore.getWholeBodyFeedbackControllerDataHolder();

      handWeight.set(20.0);
      footWeight.set(200.0);
      momentumWeight.set(1.0);
      chestWeight.set(0.02);
      pelvisOrientationWeight.set(0.02);
      pelvisHeightWeight.set(0.02);
      privilegedWeight.set(1.0);
      privilegedConfigurationGain.set(50.0);
      privilegedMaxVelocity.set(Double.POSITIVE_INFINITY);

      DoubleYoVariable hipReductionFactor = new DoubleYoVariable("hipLimitReductionFactor", registry);
      hipReductionFactor.set(0.05);
      DoubleYoVariable kneeReductionFactor = new DoubleYoVariable("kneeLimitReductionFactor", registry);
      DoubleYoVariable ankleReductionFactor = new DoubleYoVariable("ankleLimitReductionFactor", registry);

      legJointLimitReductionFactors.put(LegJointName.HIP_PITCH, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_ROLL, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_YAW, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.KNEE_PITCH, kneeReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_PITCH, ankleReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_ROLL, ankleReductionFactor);

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

   private YoGraphicCoordinateSystem createCoodinateSystem(RigidBody endEffector, Type type, AppearanceDefinition appearanceDefinition)
   {
      String namePrefix = endEffector.getName() + type.getName();
      return new YoGraphicCoordinateSystem(namePrefix, "", registry, 0.2, appearanceDefinition);
   }

   private final FeedbackControlCommandList activeCommands = new FeedbackControlCommandList();

   @Override
   protected void updateInternal()
   {
      updateTools();

      controllerCoreCommand.clear();

      registerDefaultCommands(controllerCoreCommand);
      controllerCoreCommand.addFeedbackControlCommand(consumeCommands());
      activeCommands.set(controllerCoreCommand.getFeedbackControlCommandList());
      calculateSolutionQuality(activeCommands);

      controllerCore.reset();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      updateDesiredFullRobotModelState();
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

   @Override
   protected boolean initialize()
   {
      return initializeDesiredFullRobotModelToActual();
   }

   private FeedbackControlCommandList consumeCommands()
   {
      if (commandInputManager.isNewCommandAvailable(TrackingWeightsCommand.class))
      {
         TrackingWeightsCommand command = commandInputManager.pollNewestCommand(TrackingWeightsCommand.class);
         handWeight.set(command.handWeight);
         footWeight.set(command.footWeight);
         momentumWeight.set(command.momentumWeight);
         chestWeight.set(command.chestWeight);
         pelvisOrientationWeight.set(command.pelvisOrientationWeight);
         privilegedWeight.set(command.privilegedWeight);
         privilegedConfigurationGain.set(command.privilegedConfigurationGain);
         privilegedMaxVelocity.set(command.privilegedMaxVelocity);
      }

      if (commandInputManager.isNewCommandAvailable(HandTrajectoryCommand.class))
      {
         List<HandTrajectoryCommand> commands = commandInputManager.pollNewCommands(HandTrajectoryCommand.class);

         for (int i = 0; i < commands.size(); i++)
         {
            HandTrajectoryCommand command = commands.get(i);
            RobotSide robotSide = command.getRobotSide();
            FramePose desiredPose = new FramePose();
            command.getLastTrajectoryPoint().getPoseIncludingFrame(desiredPose);
            FramePose controlFramePose = new FramePose(desiredFullRobotModel.getHandControlFrame(robotSide));

            SpatialFeedbackControlCommand handFeedbackCommand = handFeedbackCommands.get(robotSide);
            handFeedbackCommand.setGains((SE3PIDGainsInterface) gains);
            handFeedbackCommand.setWeightForSolver(handWeight.getDoubleValue());
            handFeedbackCommand.setSelectionMatrix(command.getSelectionMatrix());
            handFeedbackCommand.set(desiredPose);
            handFeedbackCommand.changeFrameAndSetControlFrameFixedInEndEffector(controlFramePose);
            activeFeedbackControlCommands.add(handFeedbackCommand);
         }
      }

      if (commandInputManager.isNewCommandAvailable(ChestTrajectoryCommand.class))
      {
         ChestTrajectoryCommand command = commandInputManager.pollNewestCommand(ChestTrajectoryCommand.class);
         FrameOrientation desiredChestOrientation = new FrameOrientation();
         command.getLastTrajectoryPoint().getOrientation(desiredChestOrientation);

         chestFeedbackCommand.setGains(gains);
         chestFeedbackCommand.setWeightForSolver(chestWeight.getDoubleValue());
         chestFeedbackCommand.setSelectionMatrix(command.getSelectionMatrix());
         chestFeedbackCommand.set(desiredChestOrientation);
         activeFeedbackControlCommands.add(chestFeedbackCommand);
      }

      if (commandInputManager.isNewCommandAvailable(PelvisHeightTrajectoryCommand.class))
      {
         PelvisHeightTrajectoryCommand command = commandInputManager.pollNewestCommand(PelvisHeightTrajectoryCommand.class);
         double desiredHeight = command.getLastTrajectoryPoint().getPosition();
         FramePoint desiredPosition = new FramePoint(worldFrame, 0.0, 0.0, desiredHeight);

         DenseMatrix64F heightSelectionMatrix = new DenseMatrix64F(1, 3);
         heightSelectionMatrix.set(0, 2, 1.0);

         pelvisHeightFeedbackCommand.setGains(gains);
         pelvisHeightFeedbackCommand.setWeightForSolver(pelvisHeightWeight.getDoubleValue());
         pelvisHeightFeedbackCommand.set(desiredPosition);
         pelvisHeightFeedbackCommand.setSelectionMatrix(heightSelectionMatrix);
         activeFeedbackControlCommands.add(pelvisHeightFeedbackCommand);
      }

      if (commandInputManager.isNewCommandAvailable(PelvisOrientationTrajectoryCommand.class))
      {
         PelvisOrientationTrajectoryCommand command = commandInputManager.pollNewestCommand(PelvisOrientationTrajectoryCommand.class);
         FrameOrientation desiredPelvisOrientation = new FrameOrientation(worldFrame);
         command.getLastTrajectoryPoint().getOrientation(desiredPelvisOrientation);

         pelvisOrientationFeedbackCommand.setGains(gains);
         pelvisOrientationFeedbackCommand.setWeightForSolver(pelvisOrientationWeight.getDoubleValue());
         pelvisOrientationFeedbackCommand.set(desiredPelvisOrientation);
         pelvisOrientationFeedbackCommand.setSelectionMatrix(command.getSelectionMatrix());
         activeFeedbackControlCommands.add(pelvisOrientationFeedbackCommand);
      }

      FeedbackControlCommandList inputs = new FeedbackControlCommandList();
      for (FeedbackControlCommand<?> feedbackControlCommand : activeFeedbackControlCommands)
         inputs.addCommand(feedbackControlCommand);
      return inputs;
   }

   public void updateTools()
   {
      desiredFullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      twistCalculator.compute();
   }

   private void registerDefaultCommands(ControllerCoreCommand controllerCoreCommand)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         SpatialFeedbackControlCommand footFeedbackCommand = footFeedbackCommands.get(robotSide);
         footFeedbackCommand.setGains((SE3PIDGainsInterface) gains);
         footFeedbackCommand.setWeightForSolver(footWeight.getDoubleValue());
         controllerCoreCommand.addFeedbackControlCommand(footFeedbackCommand);
      }

      FramePoint desiredCoM = new FramePoint(desiredCenterOfMassReference.get());
      if (desiredCoM != null)
      {
         centerOfMassFeedbackCommand.setGains(gains);
         centerOfMassFeedbackCommand.setWeightForSolver(momentumWeight.getDoubleValue());
         desiredCoM.changeFrame(worldFrame);
         centerOfMassFeedbackCommand.set(desiredCoM);
         centerOfMassFeedbackCommand.setSelectionMatrixForLinearXYControl();
         controllerCoreCommand.addFeedbackControlCommand(centerOfMassFeedbackCommand);
      }

      controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommandReference.getAndSet(null));

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

      controllerCoreCommand.addInverseKinematicsCommand(jointLimitReductionCommand);
   }

   private void updateDesiredFullRobotModelState()
   {
      RootJointDesiredConfigurationDataReadOnly outputForRootJoint = controllerCore.getOutputForRootJoint();
      desiredRootJoint.setConfiguration(outputForRootJoint.getDesiredConfiguration(), 0);
      desiredRootJoint.setVelocity(outputForRootJoint.getDesiredVelocity(), 0);

      LowLevelOneDoFJointDesiredDataHolderReadOnly output = controllerCore.getOutputForLowLevelController();
      for (OneDoFJoint joint : oneDoFJoints)
      {
         if (output.hasDataForJoint(joint))
         {
            joint.setQ(output.getDesiredJointPosition(joint));
            joint.setqDesired(output.getDesiredJointPosition(joint));
            joint.setQd(output.getDesiredJointVelocity(joint));
         }
      }
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

   private final DenseMatrix64F zeroVelocityMatrix = new DenseMatrix64F(6, 1);

   public boolean initializeDesiredFullRobotModelToActual()
   {
      activeFeedbackControlCommands.clear();

      RobotConfigurationData robotConfigurationData = latestRobotConfigurationDataReference.get();
      if (robotConfigurationData == null)
         return false;

      float[] newJointAngles = robotConfigurationData.getJointAngles();

      for (int i = 0; i < newJointAngles.length; i++)
      {
         oneDoFJoints[i].setQ(newJointAngles[i]);
         oneDoFJoints[i].setQd(0.0);
      }

      Vector3D32 translation = robotConfigurationData.getPelvisTranslation();
      desiredRootJoint.setPosition(translation.getX(), translation.getY(), translation.getZ());
      Quaternion32 orientation = robotConfigurationData.getPelvisOrientation();
      desiredRootJoint.setRotation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
      desiredRootJoint.setVelocity(zeroVelocityMatrix, 0);

      updateTools();

      FramePoint initialCoM = new FramePoint(referenceFrames.getCenterOfMassFrame());
      initialCoM.changeFrame(referenceFrames.getMidFeetZUpFrame());
      desiredCenterOfMassReference.set(initialCoM);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);
         FramePose initialFootPose = new FramePose(foot.getBodyFixedFrame());
         initialFootPose.changeFrame(worldFrame);
         SpatialFeedbackControlCommand footFeedbackCommand = footFeedbackCommands.get(robotSide);
         footFeedbackCommand.set(initialFootPose);
         footFeedbackCommand.setSelectionMatrixToIdentity();
      }

      PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_CURRENT);
      privilegedConfigurationCommand.setDefaultWeight(privilegedWeight.getDoubleValue());
      privilegedConfigurationCommand.setDefaultConfigurationGain(privilegedConfigurationGain.getDoubleValue());
      privilegedConfigurationCommand.setDefaultMaxVelocity(privilegedMaxVelocity.getDoubleValue());
      privilegedConfigurationCommandReference.set(privilegedConfigurationCommand);

      return true;
   }

   private void calculateSolutionQuality(FeedbackControlCommandList activeCommands)
   {
      double error = 0.0;

      for (int i = 0; i < activeCommands.getNumberOfCommands(); i++)
      {
         FeedbackControlCommand<?> command = activeCommands.getCommand(i);

         switch (command.getCommandType())
         {
         case MOMENTUM:
            error += calculateCommandQuality((CenterOfMassFeedbackControlCommand) command);
            break;
         case TASKSPACE:
            error += calculateCommandQuality(((SpatialFeedbackControlCommand) command).getSpatialAccelerationCommand());
            break;
         case ORIENTATION:
            error += calculateCommandQuality(((OrientationFeedbackControlCommand) command).getSpatialAccelerationCommand());
            break;
         case POINT:
            error += calculateCommandQuality(((PointFeedbackControlCommand) command).getSpatialAccelerationCommand());
            break;
         default:
            break;
         }
      }

      solutionQuality.set(error);
   }

   private final PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);

   private double calculateCommandQuality(CenterOfMassFeedbackControlCommand command)
   {
      FrameVector positionError = new FrameVector();
      feedbackControllerDataHolder.getCenterOfMassVectorData(positionError, Type.ERROR, Space.POSITION);

      DenseMatrix64F selectionMatrix = command.getMomentumRateCommand().getSelectionMatrix();
      DenseMatrix64F weightVector = command.getMomentumRateCommand().getWeightVector();

      DenseMatrix64F error = new DenseMatrix64F(6, 1);
      positionError.get(3, error);

      return computeQualityFromError(error, weightVector, selectionMatrix);
   }

   private double calculateCommandQuality(SpatialAccelerationCommand command)
   {
      boolean hasData;
      RigidBody endEffector = command.getEndEffector();

      controlFrame.setPoseAndUpdate(endEffector.getBodyFixedFrame().getTransformToRoot());
      FramePoint currentPosition = new FramePoint();
      hasData = feedbackControllerDataHolder.getPositionData(endEffector, currentPosition, Type.CURRENT);
      if (hasData)
      {
         currentPosition.changeFrame(worldFrame);
         controlFrame.setPositionAndUpdate(currentPosition);
      }
      FrameOrientation currentOrientation = new FrameOrientation();
      hasData = feedbackControllerDataHolder.getOrientationData(endEffector, currentOrientation, Type.CURRENT);
      if (hasData)
      {
         currentOrientation.changeFrame(worldFrame);
         controlFrame.setOrientationAndUpdate(currentOrientation);
      }

      FrameVector rotationError = new FrameVector();
      hasData = feedbackControllerDataHolder.getVectorData(endEffector, rotationError, Type.ERROR, Space.ROTATION_VECTOR);
      if (hasData)
         rotationError.changeFrame(controlFrame);
      else
         rotationError.setToZero(controlFrame);

      FrameVector positionError = new FrameVector();
      hasData = feedbackControllerDataHolder.getVectorData(endEffector, positionError, Type.ERROR, Space.POSITION);
      if (hasData)
         positionError.changeFrame(controlFrame);
      else
         positionError.setToZero(controlFrame);

      DenseMatrix64F selectionMatrix = command.getSelectionMatrix();
      DenseMatrix64F weightVector = command.getWeightVector();

      DenseMatrix64F error = new DenseMatrix64F(6, 1);
      rotationError.get(0, error);
      positionError.get(3, error);

      return computeQualityFromError(error, weightVector, selectionMatrix);
   }

   private double computeQualityFromError(DenseMatrix64F error, DenseMatrix64F weightVector, DenseMatrix64F selectionMatrix)
   {
      DenseMatrix64F weightMatrix = new DenseMatrix64F(6, 6);
      for (int i = 0; i < 6; i++)
         weightMatrix.set(i, i, weightVector.get(i, 0));

      DenseMatrix64F errorWeighted = new DenseMatrix64F(error.getNumRows(), 1);
      CommonOps.mult(weightMatrix, error, errorWeighted);

      DenseMatrix64F errorSubspace = new DenseMatrix64F(selectionMatrix.getNumRows(), 1);
      CommonOps.mult(selectionMatrix, errorWeighted, errorSubspace);
      return NormOps.normP2(errorSubspace);
   }

   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<>(null);

   public PacketConsumer<RobotConfigurationData> createRobotConfigurationDataConsumer()
   {
      return new PacketConsumer<RobotConfigurationData>()
      {
         @Override
         public void receivedPacket(RobotConfigurationData packet)
         {
            if (packet == null)
               return;

            latestRobotConfigurationDataReference.set(packet);
         }
      };
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