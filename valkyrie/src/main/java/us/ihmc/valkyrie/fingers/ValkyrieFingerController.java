package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.ValkyrieHandFingerTrajectoryMessageSubscriber;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlFingerStateEstimator;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ValkyrieFingerController implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoDouble time;

   private final SideDependentList<HandDesiredConfigurationMessageSubscriber> handDesiredConfigurationMessageSubscribers = new SideDependentList<>();
   private final SideDependentList<ValkyrieHandFingerTrajectoryMessageSubscriber> valkyrieHandFingerTrajectoryMessageSubscribers = new SideDependentList<>();
   private final SideDependentList<ValkyrieFingerSetController> fingerSetControllers = new SideDependentList<>();

   public ValkyrieFingerController(YoDouble yoTime, double controlDT, ValkyrieRosControlFingerStateEstimator fingerStateEstimator,
                                   List<YoEffortJointHandleHolder> jointHandles, YoVariableRegistry parentRegistry)
   {
      time = yoTime;
      YoPIDGains thumbRollGains = new YoPIDGains("HandThumbRoll", registry);
      thumbRollGains.setKp(7.0);
      thumbRollGains.setKi(3.0);
      thumbRollGains.setKd(0.0);
      thumbRollGains.setMaximumFeedback(3.0);
      thumbRollGains.setIntegralLeakRatio(0.999);
      thumbRollGains.setMaximumIntegralError(0.5);
      YoPIDGains defaultGains = new YoPIDGains("Hand", registry);
      defaultGains.setKp(7.0);
      defaultGains.setKi(3.0);
      defaultGains.setKd(0.0);
      defaultGains.setMaximumFeedback(3.0);
      defaultGains.setIntegralLeakRatio(0.999);
      defaultGains.setMaximumIntegralError(0.5);

      EnumMap<ValkyrieFingerMotorName, PIDGainsReadOnly> gains = new EnumMap<>(ValkyrieFingerMotorName.class);
      for (ValkyrieFingerMotorName motorName : ValkyrieFingerMotorName.values)
         gains.put(motorName, defaultGains);
      gains.put(ValkyrieFingerMotorName.ThumbMotorRoll, thumbRollGains);

      Map<String, YoEffortJointHandleHolder> jointHandleMap = jointHandles.stream().collect(Collectors.toMap(h -> h.getName(), h -> h));

      for (RobotSide robotSide : RobotSide.values)
      {
         HandDesiredConfigurationMessageSubscriber subscriber = new HandDesiredConfigurationMessageSubscriber(robotSide);
         handDesiredConfigurationMessageSubscribers.put(robotSide, subscriber);
         ValkyrieHandFingerTrajectoryMessageSubscriber valkyrieHandFingerTrajectoryMessageSubscriber = new ValkyrieHandFingerTrajectoryMessageSubscriber(robotSide);
         valkyrieHandFingerTrajectoryMessageSubscribers.put(robotSide, valkyrieHandFingerTrajectoryMessageSubscriber);

         EnumMap<ValkyrieFingerMotorName, YoEffortJointHandleHolder> jointHandleEnumMap = new EnumMap<>(ValkyrieFingerMotorName.class);
         for (ValkyrieFingerMotorName jointEnum : ValkyrieFingerMotorName.values)
         {
            YoEffortJointHandleHolder handle = jointHandleMap.get(jointEnum.getJointName(robotSide));
            if (handle != null)
               jointHandleEnumMap.put(jointEnum, handle);
         }

         if (!jointHandleEnumMap.isEmpty())
         {
            ValkyrieFingerSetController controller = new ValkyrieFingerSetController(robotSide,
                                                                                     time,
                                                                                     controlDT,
                                                                                     fingerStateEstimator,
                                                                                     gains,
                                                                                     jointHandleEnumMap,
                                                                                     registry);
            fingerSetControllers.put(robotSide, controller);
         }
      }
      parentRegistry.addChild(registry);
   }

   public void setupCommunication(String robotName, RealtimeRos2Node realtimeRos2Node)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ROS2Tools.createCallbackSubscription(realtimeRos2Node,
                                              HandDesiredConfigurationMessage.class,
                                              ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName),
                                              handDesiredConfigurationMessageSubscribers.get(robotSide));
         ROS2Tools.createCallbackSubscription(realtimeRos2Node,
                                              ValkyrieHandFingerTrajectoryMessage.class,
                                              ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName),
                                              valkyrieHandFingerTrajectoryMessageSubscribers.get(robotSide));
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
      checkForNewValkyrieHandFingerTrajectoryRequested();

      for (RobotSide robotSide : RobotSide.values)
      {
         ValkyrieFingerSetController controller = fingerSetControllers.get(robotSide);
         if (controller != null)
            controller.doControl();
      }
   }

   private void checkForNewValkyrieHandFingerTrajectoryRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (valkyrieHandFingerTrajectoryMessageSubscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            ValkyrieHandFingerTrajectoryMessage handFingerTrajectoryMessage = valkyrieHandFingerTrajectoryMessageSubscribers.get(robotSide).pollMessage();

            ValkyrieFingerSetController controller = fingerSetControllers.get(robotSide);
            if (controller == null)
               continue;

            controller.getHandFingerTrajectoryMessage(handFingerTrajectoryMessage);
         }
      }
   }

   private void checkForNewHandDesiredConfigurationRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handDesiredConfigurationMessageSubscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            HandConfiguration handDesiredConfiguration = HandConfiguration.fromByte(handDesiredConfigurationMessageSubscribers.get(robotSide).pollMessage()
                                                                                                                              .getDesiredHandConfiguration());
            ValkyrieFingerSetController controller = fingerSetControllers.get(robotSide);
            if (controller == null)
               continue;

            controller.getDesiredHandConfiguration(handDesiredConfiguration);
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

   public void goToInitialConfiguration()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ValkyrieFingerSetController controller = fingerSetControllers.get(robotSide);
         controller.initializeDesiredTrajectoryGenerator();
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         ValkyrieHandFingerTrajectoryMessage handFingerTrajectoryMessage = new ValkyrieHandFingerTrajectoryMessage();

         HandConfiguration handConfiguration = HandConfiguration.OPEN;
         ValkyrieHandFingerTrajectoryMessageConversion.convertHandConfiguration(robotSide, handConfiguration, handFingerTrajectoryMessage);

         ValkyrieFingerSetController controller = fingerSetControllers.get(robotSide);
         if (controller == null)
            continue;

         controller.getHandFingerTrajectoryMessage(handFingerTrajectoryMessage);
      }
   }
}
