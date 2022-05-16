package us.ihmc.avatar;

import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.avatar.stepAdjustment.PlanarRegionFootstepSnapper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ComponentBasedFootstepDataMessageGenerator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ComponentBasedFootstepDataMessageGeneratorFactory;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.Arrays;

public class AvatarStepGeneratorThread implements AvatarControllerThreadInterface
{
   private final YoRegistry csgRegistry = new YoRegistry("csgRegistry");

   private final FullHumanoidRobotModel fullRobotModel;

   private final HumanoidRobotContextData humanoidRobotContextData;
   private final HumanoidReferenceFrames humanoidReferenceFrames;
   private final YoBoolean firstTick = new YoBoolean("FirstTick", csgRegistry);
   private final YoLong timestampOffset = new YoLong("TimestampOffsetCSG", csgRegistry);
   private final YoDouble csgTime = new YoDouble("csgTime", csgRegistry);
   private final YoLong timestamp = new YoLong("TimestampCSG", csgRegistry);
   private final YoBoolean runCSG = new YoBoolean("RunCSG", csgRegistry);

   private final AvatarStepGeneratorController stepGeneratorController;
   private final CommandInputManager csgCommandInputManager;
   private final StatusMessageOutputManager walkingOutputManager;

   public AvatarStepGeneratorThread(ComponentBasedFootstepDataMessageGeneratorFactory csgPluginFactory,
                                    HumanoidRobotContextDataFactory contextDataFactory,
                                    StatusMessageOutputManager walkingOutputManager,
                                    CommandInputManager walkingCommandInputManager,
                                    DRCRobotModel drcRobotModel,
                                    double perceptionDt)
   {
      this.fullRobotModel = drcRobotModel.createFullRobotModel();
      this.walkingOutputManager = walkingOutputManager;

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
      humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();
      csgCommandInputManager = csgPluginFactory.getCSGCommandInputManager().getCommandInputManager();

      humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      ComponentBasedFootstepDataMessageGenerator csg = csgPluginFactory.buildPlugin(humanoidReferenceFrames,
                                                                                    perceptionDt,
                                                                                    drcRobotModel.getWalkingControllerParameters(),
                                                                                    walkingOutputManager,
                                                                                    walkingCommandInputManager,
                                                                                    null,
                                                                                    null,
                                                                                    csgTime);

      stepGeneratorController = new AvatarStepGeneratorController(csg,
                                                                  csgCommandInputManager,
                                                                  drcRobotModel.getWalkingControllerParameters().getSteppingParameters(),
                                                                  csgTime);

      csgRegistry.addChild(csg.getRegistry());
      csgRegistry.addChild(stepGeneratorController.getYoRegistry());

      ParameterLoaderHelper.loadParameters(this, drcRobotModel, csgRegistry);

   }

   public void createControllerNetworkSubscriber(String robotName, RealtimeROS2Node realtimeROS2Node)
   {
      ROS2Topic<?> inputTopic = ROS2Tools.getControllerInputTopic(robotName);
      ROS2Topic<?> outputTopic = ROS2Tools.getControllerOutputTopic(robotName);

      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(inputTopic,
                                                                                                csgCommandInputManager,
                                                                                                outputTopic,
                                                                                                walkingOutputManager,
                                                                                                realtimeROS2Node);

      controllerNetworkSubscriber.addMessageCollectors(ControllerAPIDefinition.createDefaultMessageIDExtractor(), 3);
      controllerNetworkSubscriber.addMessageValidator(ControllerAPIDefinition.createDefaultMessageValidation());
   }

   public void initialize()
   {
      firstTick.set(true);
      humanoidRobotContextData.setControllerRan(false);
      humanoidRobotContextData.setEstimatorRan(false);
   }

   private void runOnFirstTick()
   {
   }

   @Override
   public void run()
   {
      runCSG.set(humanoidRobotContextData.getEstimatorRan());
      if (!runCSG.getValue())
      {
         return;
      }

      try
      {
         HumanoidRobotContextTools.updateRobot(fullRobotModel, humanoidRobotContextData.getProcessedJointData());
         humanoidReferenceFrames.updateFrames();

         timestamp.set(humanoidRobotContextData.getTimestamp());
         if (firstTick.getValue())
         {
            // Record this to have time start at 0.0 on the real robot for viewing pleasure.
            timestampOffset.set(timestamp.getValue());
         }
         csgTime.set(Conversions.nanosecondsToSeconds(timestamp.getValue() - timestampOffset.getValue()));

         if (firstTick.getValue())
         {
            runOnFirstTick();
            firstTick.set(false);
         }

         stepGeneratorController.doControl();

         humanoidRobotContextData.setPerceptionRan(true);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }



   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return csgRegistry;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
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

}
