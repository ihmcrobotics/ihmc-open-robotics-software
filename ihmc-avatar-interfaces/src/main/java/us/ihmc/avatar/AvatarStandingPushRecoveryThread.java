package us.ihmc.avatar;

import controller_msgs.msg.dds.HandContactMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.multiContact.pushRecovery.ReducedOrderRobotModel;
import us.ihmc.avatar.multiContact.pushRecovery.StandingReactiveBracingPlanner;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StandingPushRecoveryAPIDefinition;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandContactCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.Arrays;

public class AvatarStandingPushRecoveryThread implements AvatarControllerThreadInterface
{
   private final YoRegistry registry = new YoRegistry("standingPushRecoveryRegistry");

   private final FullHumanoidRobotModel fullRobotModel;

   private final HumanoidRobotContextData humanoidRobotContextData;
   private final HumanoidReferenceFrames humanoidReferenceFrames;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;

   private final CommandInputManager walkingCommandInputManager;
   private final StatusMessageOutputManager walkingOutputManager;

   private final YoBoolean isHandRecoveryContactEnabled = new YoBoolean("isHandRecoveryContactEnabled", registry);
   private final YoBoolean isFalling = new YoBoolean("isFalling", registry);
   private final YoBoolean sendHandContactMessage = new YoBoolean("sendHandContactMessage", registry);
   private boolean hasSentHandTrajectory = false;
   private final StandingReactiveBracingPlanner planner;

   private final SideDependentList<HandContactCommand> plannedHandContacts = new SideDependentList<>();
   private final ReducedOrderRobotModel reducedOrderRobotModel;

   public AvatarStandingPushRecoveryThread(ROS2Node ros2Node,
                                           DRCRobotModel robotModel,
                                           HumanoidRobotContextDataFactory contextDataFactory,
                                           StatusMessageOutputManager walkingOutputManager,
                                           CommandInputManager walkingCommandInputManager)
   {
      this.fullRobotModel = robotModel.createFullRobotModel();

      String robotName = robotModel.getSimpleRobotName();
      ROS2Topic<?> inputTopic = StandingPushRecoveryAPIDefinition.getInputTopic(robotName);
      ROS2Topic<?> outputTopic = StandingPushRecoveryAPIDefinition.getOutputTopic(robotName);

      planner = robotModel.getReactiveBracingPlanner();
      reducedOrderRobotModel = new ReducedOrderRobotModel(robotModel.getContactPointParameters(), registry);

      this.commandInputManager = new CommandInputManager(StandingPushRecoveryAPIDefinition.getPushRecoverySupportedCommands());
      this.statusOutputManager = new StatusMessageOutputManager(StandingPushRecoveryAPIDefinition.getPushRecoverySupportedStatusMessages());
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

      this.centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator(), ReferenceFrame.getWorldFrame());

      isHandRecoveryContactEnabled.set(true);
   }

   public void initialize()
   {
      humanoidRobotContextData.setControllerRan(false);
      humanoidRobotContextData.setEstimatorRan(false);
   }

   @Override
   public void run()
   {
      if (!humanoidRobotContextData.getEstimatorRan())
         return;

      HumanoidRobotContextTools.updateRobot(fullRobotModel, humanoidRobotContextData.getProcessedJointData());
      humanoidReferenceFrames.updateFrames();
      centerOfMassJacobian.reset();

      FrameVector3DReadOnly comVelocity = centerOfMassJacobian.getCenterOfMassVelocity();
      isFalling.set(EuclidCoreTools.norm(comVelocity.getX(), comVelocity.getY()) > 0.07); // TODO make this better

      FramePoint3D centerOfMass = new FramePoint3D(humanoidReferenceFrames.getCenterOfMassFrame());
      FramePoint3D shoulder = new FramePoint3D(fullRobotModel.getOneDoFJointByName("LEFT_SHOULDER_Y").getFrameBeforeJoint());

      centerOfMass.changeFrame(humanoidReferenceFrames.getMidFeetZUpFrame());
      shoulder.changeFrame(humanoidReferenceFrames.getMidFeetZUpFrame());

      FrameVector3D comToShoulder = new FrameVector3D(humanoidReferenceFrames.getMidFeetZUpFrame());
      comToShoulder.sub(shoulder, centerOfMass);

      if (commandInputManager.isNewCommandAvailable(PlanarRegionsListCommand.class))
      {
         PlanarRegionsListCommand planarRegionsListCommand = commandInputManager.pollNewestCommand(PlanarRegionsListCommand.class);
//         LogTools.info("Received planar regions command! number of regions: " + planarRegionsListCommand.getNumberOfPlanarRegions());
         planner.setPlanarRegions(planarRegionsListCommand);
      }

      if (!hasSentHandTrajectory && isFalling.getValue() && isHandRecoveryContactEnabled.getValue())
      {
         hasSentHandTrajectory = true;
         sendHandContactMessage.set(false);

         reducedOrderRobotModel.initialize(fullRobotModel, humanoidReferenceFrames, comVelocity);

         plannedHandContacts.clear();
         planner.plan(reducedOrderRobotModel, plannedHandContacts);

         for (RobotSide robotSide : RobotSide.values)
         {
            HandContactCommand handContactCommand = plannedHandContacts.get(robotSide);
            if (handContactCommand != null)
            {
               walkingCommandInputManager.submitCommand(handContactCommand);
            }
         }
      }
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
