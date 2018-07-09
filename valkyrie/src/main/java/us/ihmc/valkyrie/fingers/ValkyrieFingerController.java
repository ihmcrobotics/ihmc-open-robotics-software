package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.valkyrie.fingers.ValkyrieFingerSetController.GraspState;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlFingerStateEstimator;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieFingerController implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoDouble time;
   private final YoDouble trajectoryTime = new YoDouble("fingerTrajectoryTime", registry);
   private final YoDouble thumbCloseDelay = new YoDouble("thumbCloseDelay", registry);
   private final YoDouble fingerOpenDelay = new YoDouble("fingerOpenDelay", registry);

   private final SideDependentList<HandDesiredConfigurationMessageSubscriber> subscribers = new SideDependentList<>();
   private final SideDependentList<ValkyrieFingerSetController> fingerSetControllers = new SideDependentList<>();

   public ValkyrieFingerController(YoDouble yoTime, double controlDT, ValkyrieRosControlFingerStateEstimator fingerStateEstimator,
                                   List<YoEffortJointHandleHolder> jointHandles, YoVariableRegistry parentRegistry)
   {
      time = yoTime;
      trajectoryTime.set(5.0); // The fingers seem to be pretty slow, so kinda pointless reducing this one.
      thumbCloseDelay.set(1.25); // Making sure the thumb does not crush the finger when closing.
      fingerOpenDelay.set(0.25); // Assuming the thumb is over the fingers, probably better to start opening it first.

      YoPIDGains gains = new YoPIDGains("Hand", registry);
      gains.setKp(7.0);
      gains.setKi(3.0);
      gains.setKd(0.0);
      gains.setMaximumFeedback(3.0);
      gains.setIntegralLeakRatio(0.999);
      gains.setMaximumIntegralError(0.5);

      Map<String, YoEffortJointHandleHolder> jointHandleMap = jointHandles.stream().collect(Collectors.toMap(h -> h.getName(), h -> h));

      for (RobotSide robotSide : RobotSide.values)
      {
         HandDesiredConfigurationMessageSubscriber subscriber = new HandDesiredConfigurationMessageSubscriber(robotSide);
         subscribers.put(robotSide, subscriber);

         EnumMap<ValkyrieFingerMotorName, YoEffortJointHandleHolder> jointHandleEnumMap = new EnumMap<>(ValkyrieFingerMotorName.class);
         for (ValkyrieFingerMotorName jointEnum : ValkyrieFingerMotorName.values)
         {
            YoEffortJointHandleHolder handle = jointHandleMap.get(jointEnum.getJointName(robotSide));
            if (handle != null)
               jointHandleEnumMap.put(jointEnum, handle);
         }

         if (!jointHandleEnumMap.isEmpty())
         {
            ValkyrieFingerSetController controller = new ValkyrieFingerSetController(robotSide, time, controlDT, fingerStateEstimator, gains, trajectoryTime,
                                                                                     thumbCloseDelay, fingerOpenDelay, jointHandleEnumMap, registry);
            fingerSetControllers.put(robotSide, controller);
         }
      }
      parentRegistry.addChild(registry);
   }

   public void setupCommunication(String robotName, RealtimeRos2Node realtimeRos2Node)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ROS2Tools.createCallbackSubscription(realtimeRos2Node, HandDesiredConfigurationMessage.class,
                                              ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName), subscribers.get(robotSide));
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      checkForNewHandDesiredConfigurationRequested();

      for (RobotSide robotSide : RobotSide.values)
      {
         ValkyrieFingerSetController controller = fingerSetControllers.get(robotSide);
         if (controller != null)
            controller.doControl();
      }
   }

   private void checkForNewHandDesiredConfigurationRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (subscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            HandConfiguration handDesiredConfiguration = HandConfiguration.fromByte(subscribers.get(robotSide).pollMessage().getDesiredHandConfiguration());
            ValkyrieFingerSetController controller = fingerSetControllers.get(robotSide);
            if (controller == null)
               continue;

            switch (handDesiredConfiguration)
            {
            case OPEN:
               controller.requestState(GraspState.OPEN);
               break;

            case CLOSE:
               controller.requestState(GraspState.CLOSE);
               break;

            default:
               break;
            }
         }
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public String getName()
   {
      return name;
   }
}
