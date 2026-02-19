package us.ihmc.avatar;

import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MultiContactGaitGenerator;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

public class AvatarMultiContactGaitGeneratorThread implements AvatarControllerThreadInterface
{
   private static final int NUMBER_OF_PREVIEW_STEPS = 3;

   private final YoRegistry registry = new YoRegistry("multiContactGaitGeneratorThread");
   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidRobotContextData humanoidRobotContextData;
   private final HumanoidReferenceFrames humanoidReferenceFrames;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;

   private final CommandInputManager walkingCommandInputManager;
   private final StatusMessageOutputManager walkingOutputManager;

   private final YoBoolean walkPrev = new YoBoolean("walkMCGSPrev", registry);
   private final YoBoolean walk = new YoBoolean("walkMCGS", registry);
   private final YoDouble nominalSwingDuration = new YoDouble("nominalSwingDuration", registry);
   private final YoDouble nominalTransferDuration = new YoDouble("nominalTransferDuration", registry);
   private final YoDouble desiredWalkingVelocityX = new YoDouble("desiredWalkingVelocityX", registry);
   private final YoDouble desiredWalkingVelocityY = new YoDouble("desiredWalkingVelocityY", registry);
   private final YoDouble desiredWalkingVelocityYaw = new YoDouble("desiredWalkingVelocityYaw", registry);

   private final YoEnum<RobotSide> currentSwingSide = new YoEnum<>("currentSwingSide", registry, RobotSide.class, true);
   private final YoFramePose3D currentTouchdownPose = new YoFramePose3D("currentTouchdownPose", ReferenceFrame.getWorldFrame(), registry);
   private final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>();

   private final double stanceWidth;
   private final FramePose3D tempPose = new FramePose3D();
   private final FootstepDataListCommand footstepDataListCommand = new FootstepDataListCommand();

   public AvatarMultiContactGaitGeneratorThread(ROS2Node ros2Node,
                                                DRCRobotModel robotModel,
                                                HumanoidRobotContextDataFactory contextDataFactory,
                                                StatusMessageOutputManager walkingOutputManager,
                                                CommandInputManager walkingCommandInputManager)
   {
      this.fullRobotModel = robotModel.createFullRobotModel();

      String robotName = robotModel.getSimpleRobotName();
      ROS2Topic<?> inputTopic = MultiContactGaitGenerator.getInputTopic(robotName);
      ROS2Topic<?> outputTopic = MultiContactGaitGenerator.getOutputTopic(robotName);

      this.commandInputManager = new CommandInputManager(MultiContactGaitGenerator.getSupportedCommands());
      this.statusOutputManager = new StatusMessageOutputManager(MultiContactGaitGenerator.getSupportedStatusMessages());
      new ControllerNetworkSubscriber(inputTopic, commandInputManager, outputTopic, statusOutputManager, ros2Node);

      HumanoidRobotContextJointData processedJointData = new HumanoidRobotContextJointData(fullRobotModel.getOneDoFJoints().length);
      ForceSensorDataHolder forceSensorDataHolderForController = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      CenterOfMassDataHolder centerOfMassDataHolderForController = new CenterOfMassDataHolder();
      CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator = new CenterOfPressureDataHolder(fullRobotModel);
      LowLevelOneDoFJointDesiredDataHolder desiredJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder(fullRobotModel.getControllableOneDoFJoints());
      RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();
      contextDataFactory.setForceSensorDataHolder(forceSensorDataHolderForController);
      contextDataFactory.setCenterOfMassDataHolder(centerOfMassDataHolderForController);
      contextDataFactory.setCenterOfPressureDataHolder(centerOfPressureDataHolderForEstimator);
      contextDataFactory.setRobotMotionStatusHolder(robotMotionStatusHolder);
      contextDataFactory.setJointDesiredOutputList(desiredJointDataHolder);
      contextDataFactory.setProcessedJointData(processedJointData);
      contextDataFactory.setSensorDataContext(new SensorDataContext(fullRobotModel));
      this.humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();

      this.humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.walkingOutputManager = walkingOutputManager;
      this.walkingCommandInputManager = walkingCommandInputManager;

      statusOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, walkingStatusMessage::set);

      nominalSwingDuration.set(robotModel.getWalkingControllerParameters().getDefaultSwingTime());
      nominalTransferDuration.set(robotModel.getWalkingControllerParameters().getDefaultTransferTime());
   }

   @Override
   public void run()
   {
      if (!humanoidRobotContextData.getEstimatorRan())
         return;

      if (!isWalking())
      {
         currentSwingSide.set(null);
         currentTouchdownPose.setToNaN();
      }

      boolean walk = this.walk.getValue();
      boolean walkPrev = this.walkPrev.getValue();
      this.walkPrev.set(walk);

      if (!walk)
         return;

      HumanoidRobotContextTools.updateRobot(fullRobotModel, humanoidRobotContextData.getProcessedJointData());
      humanoidReferenceFrames.updateFrames();

      if (!walkPrev)
      {
         // Initialize based on current stance
         RobotSide initialStanceSide = RobotSide.RIGHT;
         currentSwingSide.set(initialStanceSide);
         tempPose.setToZero(humanoidReferenceFrames.getSoleFrame(initialStanceSide));
         tempPose.changeFrame(ReferenceFrame.getWorldFrame());
         currentTouchdownPose.set(tempPose);
      }

      footstepDataListCommand.getFootsteps().clear();
      for (int i = 0; i < NUMBER_OF_PREVIEW_STEPS; i++)
      {

      }
   }

   private boolean isWalking()
   {
      WalkingStatusMessage walkingStatusMessage = this.walkingStatusMessage.get();
      if (walkingStatusMessage == null)
         return false;
      WalkingStatus walkingStatus = WalkingStatus.fromByte(walkingStatusMessage.getWalkingStatus());
      return walkingStatus == WalkingStatus.STARTED || walkingStatus == WalkingStatus.RESUMED;
   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   @Override
   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      return humanoidRobotContextData;
   }

   @Override
   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      return null;
   }
}