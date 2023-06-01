package us.ihmc.valkyrie.hands.athena;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel.AthenaFingerMotorName;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlAthenaStateEstimator;
import us.ihmc.valkyrieRosControl.dataHolders.YoEffortJointHandleHolder;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import valkyrie_msgs.msg.dds.AthenaTrajectoryMessage;

public class AthenaFingerController implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final YoDouble time;

   private final SideDependentList<HandDesiredConfigurationMessageSubscriber> handDesiredConfigurationMessageSubscribers = new SideDependentList<>();
   private final SideDependentList<AthenaTrajectoryMessageSubscriber> athenaTrajectoryMessageSubscribers = new SideDependentList<>();
   private final SideDependentList<AthenaController> fingerSetControllers = new SideDependentList<>();

   public AthenaFingerController(YoDouble yoTime,
                                 double controlDT,
                                 ValkyrieRosControlAthenaStateEstimator fingerStateEstimator,
                                 List<YoEffortJointHandleHolder> jointHandles,
                                 YoRegistry parentRegistry)
   {
      time = yoTime;
      YoPIDGains thumbRollGains = new YoPIDGains("HandThumbRoll", registry);
      thumbRollGains.setKp(100.0);
      thumbRollGains.setKi(3.0);
      thumbRollGains.setKd(0.0);
      thumbRollGains.setMaximumFeedback(50.0);
      thumbRollGains.setIntegralLeakRatio(0.999);
      thumbRollGains.setMaximumIntegralError(0.5);
      YoPIDGains defaultGains = new YoPIDGains("Hand", registry);
      defaultGains.setKp(7.0);
      defaultGains.setKi(3.0);
      defaultGains.setKd(0.0);
      defaultGains.setMaximumFeedback(3.0);
      defaultGains.setIntegralLeakRatio(0.999);
      defaultGains.setMaximumIntegralError(0.5);

      EnumMap<AthenaFingerMotorName, PIDGainsReadOnly> gains = new EnumMap<>(AthenaFingerMotorName.class);
      for (AthenaFingerMotorName motorName : AthenaFingerMotorName.values)
         gains.put(motorName, defaultGains);
      gains.put(AthenaFingerMotorName.ThumbMotorRoll, thumbRollGains);

      Map<String, YoEffortJointHandleHolder> jointHandleMap = jointHandles.stream().collect(Collectors.toMap(h -> h.getName(), h -> h));

      for (RobotSide robotSide : RobotSide.values)
      {
         HandDesiredConfigurationMessageSubscriber subscriber = new HandDesiredConfigurationMessageSubscriber(robotSide);
         handDesiredConfigurationMessageSubscribers.put(robotSide, subscriber);
         AthenaTrajectoryMessageSubscriber valkyrieHandFingerTrajectoryMessageSubscriber = new AthenaTrajectoryMessageSubscriber(robotSide);
         athenaTrajectoryMessageSubscribers.put(robotSide, valkyrieHandFingerTrajectoryMessageSubscriber);

         EnumMap<AthenaFingerMotorName, YoEffortJointHandleHolder> jointHandleEnumMap = new EnumMap<>(AthenaFingerMotorName.class);
         for (AthenaFingerMotorName jointEnum : AthenaFingerMotorName.values)
         {
            YoEffortJointHandleHolder handle = jointHandleMap.get(jointEnum.getJointName(robotSide));
            if (handle != null)
               jointHandleEnumMap.put(jointEnum, handle);
         }

         if (!jointHandleEnumMap.isEmpty())
         {
            AthenaController controller = new AthenaController(robotSide, time, controlDT, fingerStateEstimator, gains, jointHandleEnumMap, registry);
            fingerSetControllers.put(robotSide, controller);
         }
      }
      parentRegistry.addChild(registry);
   }

   public void setupCommunication(String robotName, RealtimeROS2Node realtimeROS2Node)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node,
                                                       HandDesiredConfigurationMessage.class,
                                                       ROS2Tools.getControllerInputTopic(robotName),
                                                       handDesiredConfigurationMessageSubscribers.get(robotSide));
         ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node,
                                                       AthenaTrajectoryMessage.class,
                                                       ROS2Tools.getControllerInputTopic(robotName),
                                                       athenaTrajectoryMessageSubscribers.get(robotSide));
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
         AthenaController controller = fingerSetControllers.get(robotSide);
         if (controller != null)
            controller.doControl();
      }
   }

   private void checkForNewValkyrieHandFingerTrajectoryRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (athenaTrajectoryMessageSubscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            AthenaTrajectoryMessage handFingerTrajectoryMessage = athenaTrajectoryMessageSubscribers.get(robotSide).pollMessage();

            AthenaController controller = fingerSetControllers.get(robotSide);
            if (controller == null)
               continue;

            controller.getAthenaTrajectoryMessage(handFingerTrajectoryMessage);
         }
      }
   }

   private void checkForNewHandDesiredConfigurationRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handDesiredConfigurationMessageSubscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            HandConfiguration handDesiredConfiguration = HandConfiguration.fromByte(handDesiredConfigurationMessageSubscribers.get(robotSide)
                                                                                                                              .pollMessage()
                                                                                                                              .getDesiredHandConfiguration());
            AthenaController controller = fingerSetControllers.get(robotSide);
            if (controller == null)
               continue;

            controller.getDesiredHandConfiguration(handDesiredConfiguration);
         }
      }
   }

   @Override
   public YoRegistry getYoRegistry()
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
         AthenaController controller = fingerSetControllers.get(robotSide);
         controller.initializeDesiredTrajectoryGenerator();
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         AthenaTrajectoryMessage athenaTrajectoryMessage = new AthenaTrajectoryMessage();

         HandConfiguration handConfiguration = HandConfiguration.OPEN;
         AthenaTrajectoryMessageConversion.convertHandConfiguration(robotSide, handConfiguration, athenaTrajectoryMessage);

         AthenaController controller = fingerSetControllers.get(robotSide);
         if (controller == null)
            continue;

         controller.getAthenaTrajectoryMessage(athenaTrajectoryMessage);
      }
   }
}
