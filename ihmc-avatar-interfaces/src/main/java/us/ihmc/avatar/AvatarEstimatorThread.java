package us.ihmc.avatar;

import java.util.Map;
import java.util.function.BooleanSupplier;

import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ControllerCrashLocation;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorStateUpdater;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AvatarEstimatorThread extends ModularRobotController
{
   private final YoRegistry estimatorRegistry;
   private final YoBoolean firstTick;

   private final SensorReader sensorReader;
   private final FullHumanoidRobotModel estimatorFullRobotModel;
   private final StateEstimatorController mainStateEstimator;
   private final PairList<BooleanSupplier, StateEstimatorController> secondaryStateEstimators;
   private final ForceSensorStateUpdater forceSensorStateUpdater;

   private final RigidBodyTransform rootToWorldTransform = new RigidBodyTransform();

   private final HumanoidRobotContextData humanoidRobotContextData;

   private final IHMCRealtimeROS2Publisher<ControllerCrashNotificationPacket> controllerCrashPublisher;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   public AvatarEstimatorThread(SensorReader sensorReader, FullHumanoidRobotModel estimatorFullRobotModel, HumanoidRobotContextData humanoidRobotContextData,
                                   StateEstimatorController mainStateEstimator, PairList<BooleanSupplier, StateEstimatorController> secondaryStateEstimators,
                                   ForceSensorStateUpdater forceSensorStateUpdater,
                                   IHMCRealtimeROS2Publisher<ControllerCrashNotificationPacket> controllerCrashPublisher, YoRegistry estimatorRegistry,
                                   YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super("EstimatorController");

      this.sensorReader = sensorReader;
      this.estimatorFullRobotModel = estimatorFullRobotModel;
      this.humanoidRobotContextData = humanoidRobotContextData;
      this.mainStateEstimator = mainStateEstimator;
      this.secondaryStateEstimators = secondaryStateEstimators;
      this.forceSensorStateUpdater = forceSensorStateUpdater;
      this.controllerCrashPublisher = controllerCrashPublisher;
      this.estimatorRegistry = estimatorRegistry;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      // This is to preserve the registry hierarchy for the parameters to work
      estimatorRegistry.addChild(super.getYoRegistry());

      if (mainStateEstimator != null)
         addRobotController(mainStateEstimator);

      for (int i = 0; i < secondaryStateEstimators.size(); i++)
      {
         addRobotController(secondaryStateEstimators.get(i).getRight());
      }

      firstTick = new YoBoolean("firstTick", estimatorRegistry);
      firstTick.set(true);
   }

   public void run()
   {
      try
      {
         if (firstTick.getBooleanValue())
         {
            initialize();

            if (forceSensorStateUpdater != null)
               forceSensorStateUpdater.initialize();

            firstTick.set(false);
         }

         for (int i = 0; i < secondaryStateEstimators.size(); i++)
         {
            estimatorFullRobotModel.getRootJoint().getJointConfiguration(rootToWorldTransform);
            if (secondaryStateEstimators.get(i).getLeft().getAsBoolean())
               secondaryStateEstimators.get(i).getRight().initializeEstimator(rootToWorldTransform);
         }

         sensorReader.compute(humanoidRobotContextData.getTimestamp(), humanoidRobotContextData.getSensorDataContext());

         doControl();

         if (forceSensorStateUpdater != null)
         {
            forceSensorStateUpdater.updateForceSensorState();
         }

         HumanoidRobotContextTools.updateContext(estimatorFullRobotModel, humanoidRobotContextData.getProcessedJointData());
         humanoidRobotContextData.setEstimatorRan(!firstTick.getValue());
      }
      catch (Throwable e)
      {
         if (controllerCrashPublisher != null)
         {
            controllerCrashPublisher.publish(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.ESTIMATOR_RUN, e));
         }
         throw new RuntimeException(e);
      }
   }

   public void initializeStateEstimators(RigidBodyTransform rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      sensorReader.initialize();

      if (mainStateEstimator != null)
         mainStateEstimator.initializeEstimator(rootJointTransform, jointPositions);

      for (int i = 0; i < secondaryStateEstimators.size(); i++)
      {
         secondaryStateEstimators.get(i).getRight().initializeEstimator(rootJointTransform, jointPositions);
      }

      firstTick.set(true);
      humanoidRobotContextData.setControllerRan(false);
      humanoidRobotContextData.setEstimatorRan(false);
   }

   public void setupHighLevelControllerCallback(String robotName, RealtimeRos2Node realtimeRos2Node,
                                                Map<HighLevelControllerName, StateEstimatorMode> stateModeMap)
   {
      ROS2Topic outputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, outputTopic, subscriber ->
      {
         HighLevelStateChangeStatusMessage message = subscriber.takeNextData();
         StateEstimatorMode requestedMode = stateModeMap.get(HighLevelControllerName.fromByte(message.getEndHighLevelControllerName()));
         mainStateEstimator.requestStateEstimatorMode(requestedMode);
      });
   }

   @Deprecated // TODO: Split up the sensor reader and move the part needed outside out of this class!
   public SensorReader getSensorReader()
   {
      return sensorReader;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return estimatorFullRobotModel;
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return estimatorRegistry;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      return humanoidRobotContextData;
   }
}
