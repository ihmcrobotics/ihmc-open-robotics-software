package us.ihmc.avatar;

import controller_msgs.msg.dds.HandContactMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandContactCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Arrays;

public class AvatarStandingPushRecoveryThread implements AvatarControllerThreadInterface
{
   private final YoRegistry registry = new YoRegistry("standingPushRecoveryRegistry");

   private final FullHumanoidRobotModel fullRobotModel;

   private final HumanoidRobotContextData humanoidRobotContextData;
   private final HumanoidReferenceFrames humanoidReferenceFrames;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final StatusMessageOutputManager walkingOutputManager;
   private final CommandInputManager walkingCommandInputManager;

   private final YoBoolean isHandRecoveryContactEnabled = new YoBoolean("isHandRecoveryContactEnabled", registry);
   private final YoBoolean isFalling = new YoBoolean("isFalling", registry);
   private final YoBoolean sendHandContactMessage = new YoBoolean("sendHandContactMessage", registry);
   private boolean hasSentHandTrajectory = false;

   public AvatarStandingPushRecoveryThread(DRCRobotModel robotModel,
                                           HumanoidRobotContextDataFactory contextDataFactory,
                                           StatusMessageOutputManager walkingOutputManager,
                                           CommandInputManager walkingCommandInputManager)
   {
      this.fullRobotModel = robotModel.createFullRobotModel();

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
//      isFalling.set(EuclidCoreTools.norm(comVelocity.getX(), comVelocity.getY()) > 0.07); // TODO make this better

      if (!hasSentHandTrajectory && isFalling.getValue() && isHandRecoveryContactEnabled.getValue())
      {
         hasSentHandTrajectory = true;
         sendHandContactMessage.set(false);

         RobotSide robotSide = RobotSide.LEFT;
         Point3D point = new Point3D(1.041, -0.499, 1.300);
         Vector3D normal = new Vector3D(-0.887, 0.413, 0.208);

         HandContactMessage handContactMessage = new HandContactMessage();
         handContactMessage.setRobotSide(robotSide.toByte());
         handContactMessage.setTrajectoryDuration(0.24);
         handContactMessage.getBracingPoint().set(point);
         handContactMessage.getBracingNormal().set(normal);

         HandContactCommand handContactCommand = new HandContactCommand();
         handContactCommand.setFromMessage(handContactMessage);
         walkingCommandInputManager.submitCommand(handContactCommand);
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
