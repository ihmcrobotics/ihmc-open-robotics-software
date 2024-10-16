package us.ihmc.robotiq.simulatedHand;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.AvatarSimulatedHandControlThread;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commons.Conversions;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class SimulatedRobotiqHandsControlThread implements AvatarSimulatedHandControlThread
{
   private final YoRegistry registry = new YoRegistry("HandControlThread");

   private final YoDouble controllerTime = new YoDouble("HandControlTime", registry);
   private final YoBoolean firstTick = new YoBoolean("FirstHandControlTick", registry);
   private final YoBoolean runController = new YoBoolean("RunHandControl", registry);
   private final YoLong timestampOffset = new YoLong("TimestampOffsetHandControl", registry);
   private final YoLong timestamp = new YoLong("TimestampHandControl", registry);

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidRobotContextData humanoidRobotContextData;
   private final SimulatedRobotiqHandsController controller;

   private final List<OneDoFJointBasics> controlledFingerJoints = new ArrayList<>();

   public SimulatedRobotiqHandsControlThread(FullHumanoidRobotModel fullRobotModel,
                                             RealtimeROS2Node realtimeROS2Node,
                                             ROS2Topic<?> outputTopic,
                                             ROS2Topic<?> inputTopic,
                                             RobotSide[] sides)
   {
      this.fullRobotModel = fullRobotModel;

      for (RobotSide robotSide : sides)
      {
         SubtreeStreams.fromChildren(OneDoFJointBasics.class, fullRobotModel.getHand(robotSide)).forEach(controlledFingerJoints::add);
      }

      humanoidRobotContextData = new HumanoidRobotContextData(controlledFingerJoints);
      controller = new SimulatedRobotiqHandsController(fullRobotModel,
                                                       humanoidRobotContextData.getJointDesiredOutputList(),
                                                       controllerTime,
                                                       realtimeROS2Node,
                                                       outputTopic,
                                                       inputTopic,
                                                       sides);
      registry.addChild(controller.getYoRegistry());

      firstTick.set(true);
   }

   public void initialize()
   {
      firstTick.set(true);
   }

   @Override
   public void run()
   {
      runController.set(humanoidRobotContextData.getEstimatorRan());

      if (!runController.getValue())
      {
         return;
      }

      try
      {
         updateFullRobotModel();
         timestamp.set(humanoidRobotContextData.getTimestamp());
         if (firstTick.getValue())
         {
            // Record this to have time start at 0.0 on the real robot for viewing pleasure.
            timestampOffset.set(timestamp.getValue());
         }
         controllerTime.set(Conversions.nanosecondsToSeconds(timestamp.getValue() - timestampOffset.getValue()));

         if (firstTick.getValue())
         {
            controller.initialize();
            firstTick.set(false);
         }

         controller.doControl();
         humanoidRobotContextData.setControllerRan(true);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
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
   public List<OneDoFJointBasics> getControlledOneDoFJoints()
   {
      return controlledFingerJoints;
   }

   @Override
   public boolean hasControllerRan()
   {
      return runController.getValue();
   }

   @Override
   public void cleanup()
   {
      controller.cleanup();
   }
}
