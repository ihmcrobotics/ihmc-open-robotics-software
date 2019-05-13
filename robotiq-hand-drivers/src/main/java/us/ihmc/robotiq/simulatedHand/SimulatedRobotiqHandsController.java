package us.ihmc.robotiq.simulatedHand;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotiq.model.RobotiqHandModel;
import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulatedRobotiqHandsController extends HumanoidRobotControlTask
{
   private final boolean DEBUG = false;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoDouble handControllerTime = new YoDouble("handControllerTime", registry);
   private final YoBoolean sendFingerJointGains = new YoBoolean("sendFingerJointGains", registry);

   private final LinkedHashMap<OneDegreeOfFreedomJoint, YoDouble> kpMap = new LinkedHashMap<>();
   private final LinkedHashMap<OneDegreeOfFreedomJoint, YoDouble> kdMap = new LinkedHashMap<>();

   private final YoDouble fingerTrajectoryTime = new YoDouble("FingerTrajectoryTime", registry);

   private final SideDependentList<HandDesiredConfigurationMessageSubscriber> handDesiredConfigurationMessageSubscribers = new SideDependentList<>();

   private final SideDependentList<List<OneDegreeOfFreedomJoint>> allFingerJoints = new SideDependentList<>();

   private final RobotiqHandModel handModel = new RobotiqHandModel();

   private final SideDependentList<IndividualRobotiqHandController> individualHandControllers = new SideDependentList<>();

   private final SimulatedRobotiqHandJointAngleProducer jointAngleProducer;

   private final SideDependentList<Boolean> hasRobotiqHand = new SideDependentList<Boolean>(false, false);

   private long timestamp;

   public SimulatedRobotiqHandsController(FloatingRootJointRobot simulatedRobot, DRCRobotModel robotModel, RealtimeRos2Node realtimeRos2Node,
                                          MessageTopicNameGenerator pubTopicNameGenerator, MessageTopicNameGenerator subTopicNameGenerator)
   {
      super((int) Math.round(robotModel.getControllerDT() / robotModel.getSimulateDT()));

      sendFingerJointGains.set(true);

      if (realtimeRos2Node != null)
      {
         IHMCRealtimeROS2Publisher<HandJointAnglePacket> jointAnglePublisher = ROS2Tools.createPublisher(realtimeRos2Node, HandJointAnglePacket.class,
                                                                                                         pubTopicNameGenerator);
         jointAngleProducer = new SimulatedRobotiqHandJointAngleProducer(jointAnglePublisher, simulatedRobot);
      }
      else
      {
         jointAngleProducer = null;
      }
      fingerTrajectoryTime.set(0.5);

      EnumMap<RobotiqHandJointNameMinimal, YoDouble> kpEnumMap = new EnumMap<>(RobotiqHandJointNameMinimal.class);
      EnumMap<RobotiqHandJointNameMinimal, YoDouble> kdEnumMap = new EnumMap<>(RobotiqHandJointNameMinimal.class);

      setupGains(kpEnumMap, kdEnumMap);

      for (RobotSide robotSide : RobotSide.values)
      {
         allFingerJoints.put(robotSide, new ArrayList<OneDegreeOfFreedomJoint>());

         for (HandJointName jointEnum : handModel.getHandJointNames())
         {
            OneDegreeOfFreedomJoint fingerJoint = simulatedRobot.getOneDegreeOfFreedomJoint(jointEnum.getJointName(robotSide));
            if (fingerJoint != null)
               hasRobotiqHand.put(robotSide, true);
            allFingerJoints.get(robotSide).add(fingerJoint);
            kpMap.put(fingerJoint, kpEnumMap.get(jointEnum));
            kdMap.put(fingerJoint, kdEnumMap.get(jointEnum));
         }

         if (hasRobotiqHand.get(robotSide))
         {
            HandDesiredConfigurationMessageSubscriber handDesiredConfigurationSubscriber = new HandDesiredConfigurationMessageSubscriber(robotSide);
            handDesiredConfigurationMessageSubscribers.put(robotSide, handDesiredConfigurationSubscriber);
            if (realtimeRos2Node != null)
            {
               ROS2Tools.createCallbackSubscription(realtimeRos2Node, HandDesiredConfigurationMessage.class, subTopicNameGenerator,
                                                    handDesiredConfigurationSubscriber);
            }

            IndividualRobotiqHandController individualHandController = new IndividualRobotiqHandController(robotSide, handControllerTime, fingerTrajectoryTime,
                                                                                                           simulatedRobot, registry);
            individualHandControllers.put(robotSide, individualHandController);
         }
      }
   }

   private void setupGains(EnumMap<RobotiqHandJointNameMinimal, YoDouble> kpEnumMap, EnumMap<RobotiqHandJointNameMinimal, YoDouble> kdEnumMap)
   {
      YoDouble kpFingerJoint1 = new YoDouble("kpFingerJoint1", registry);
      YoDouble kpFingerJoint2 = new YoDouble("kpFingerJoint2", registry);
      YoDouble kpFingerJoint3 = new YoDouble("kpFingerJoint3", registry);
      YoDouble kpThumbJoint1 = new YoDouble("kpThumbJoint1", registry);
      YoDouble kpThumbJoint2 = new YoDouble("kpThumbJoint2", registry);
      YoDouble kpThumbJoint3 = new YoDouble("kpThumbJoint3", registry);

      kpFingerJoint1.set(10.0);
      kpFingerJoint2.set(5.0);
      kpFingerJoint3.set(1.0);

      kpThumbJoint1.set(20.0);
      kpThumbJoint2.set(10.0);
      kpThumbJoint3.set(2.0);

      kpEnumMap.put(RobotiqHandJointNameMinimal.PALM_FINGER_1_JOINT, kpFingerJoint1);
      kpEnumMap.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_1, kpFingerJoint1);
      kpEnumMap.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_2, kpFingerJoint2);
      kpEnumMap.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_3, kpFingerJoint3);

      kpEnumMap.put(RobotiqHandJointNameMinimal.PALM_FINGER_2_JOINT, kpFingerJoint1);
      kpEnumMap.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_1, kpFingerJoint1);
      kpEnumMap.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_2, kpFingerJoint2);
      kpEnumMap.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_3, kpFingerJoint3);

      kpEnumMap.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_1, kpThumbJoint1);
      kpEnumMap.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_2, kpThumbJoint2);
      kpEnumMap.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_3, kpThumbJoint3);

      YoDouble kdFingerJoint1 = new YoDouble("kdFingerJoint1", registry);
      YoDouble kdFingerJoint2 = new YoDouble("kdFingerJoint2", registry);
      YoDouble kdFingerJoint3 = new YoDouble("kdFingerJoint3", registry);
      YoDouble kdThumbJoint1 = new YoDouble("kdThumbJoint1", registry);
      YoDouble kdThumbJoint2 = new YoDouble("kdThumbJoint2", registry);
      YoDouble kdThumbJoint3 = new YoDouble("kdThumbJoint3", registry);

      kdFingerJoint1.set(0.5);
      kdFingerJoint2.set(0.25);
      kdFingerJoint3.set(0.1);

      kdThumbJoint1.set(1.0);
      kdThumbJoint2.set(0.5);
      kdThumbJoint3.set(0.2);

      kdEnumMap.put(RobotiqHandJointNameMinimal.PALM_FINGER_1_JOINT, kdFingerJoint1);
      kdEnumMap.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_1, kdFingerJoint1);
      kdEnumMap.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_2, kdFingerJoint2);
      kdEnumMap.put(RobotiqHandJointNameMinimal.FINGER_1_JOINT_3, kdFingerJoint3);

      kdEnumMap.put(RobotiqHandJointNameMinimal.PALM_FINGER_2_JOINT, kdFingerJoint1);
      kdEnumMap.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_1, kdFingerJoint1);
      kdEnumMap.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_2, kdFingerJoint2);
      kdEnumMap.put(RobotiqHandJointNameMinimal.FINGER_2_JOINT_3, kdFingerJoint3);

      kdEnumMap.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_1, kdThumbJoint1);
      kdEnumMap.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_2, kdThumbJoint2);
      kdEnumMap.put(RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_3, kdThumbJoint3);
   }

   @Override
   public boolean initialize()
   {
      return true;
   }

   @Override
   public void execute()
   {
      runInternal();
   }

   public void read()
   {
      handControllerTime.set(Conversions.nanosecondsToSeconds(timestamp));

      if (jointAngleProducer != null)
      {
         jointAngleProducer.sendHandJointAnglesPacket();
      }
   }

   public void runInternal()
   {
      checkForNewHandDesiredConfigurationRequested();

      for (RobotSide robotSide : RobotSide.values)
      {
         if (hasRobotiqHand.get(robotSide))
            individualHandControllers.get(robotSide).doControl();
      }
   }

   private void checkForNewHandDesiredConfigurationRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!hasRobotiqHand.get(robotSide))
            continue;

         HandDesiredConfigurationMessageSubscriber handDesiredConfigurationSubscriber = handDesiredConfigurationMessageSubscribers.get(robotSide);
         if (handDesiredConfigurationSubscriber.isNewDesiredConfigurationAvailable())
         {
            IndividualRobotiqHandController individualRobotiqHandController = individualHandControllers.get(robotSide);

            HandConfiguration handDesiredConfiguration = HandConfiguration.fromByte(handDesiredConfigurationSubscriber.pollMessage()
                                                                                                                      .getDesiredHandConfiguration());

            if (DEBUG)
               LogTools.debug("Recieved new HandDesiredConfiguration: " + handDesiredConfiguration);
            switch (handDesiredConfiguration)
            {
            case OPEN:
               individualRobotiqHandController.open();
               break;

            case OPEN_INDEX:
               //TODO
               break;

            case OPEN_MIDDLE:
               //TODO
               break;

            case OPEN_FINGERS:
               individualRobotiqHandController.openFingers();
               break;

            case OPEN_THUMB:
               individualRobotiqHandController.openThumb();
               break;

            case CLOSE:
               individualRobotiqHandController.close();
               break;

            case CLOSE_FINGERS:
               individualRobotiqHandController.closeFingers();
               break;

            case CLOSE_THUMB:
               individualRobotiqHandController.closeThumb();
               break;

            case RESET:
               individualRobotiqHandController.reset();
               break;

            case HOOK:
               individualRobotiqHandController.hook();
               break;

            case CRUSH:
               individualRobotiqHandController.crush();
               break;

            case CRUSH_INDEX:
               //TODO
               break;

            case CRUSH_MIDDLE:
               //TODO
               break;

            case CRUSH_THUMB:
               individualRobotiqHandController.crushThumb();
               break;

            case STOP:
               individualRobotiqHandController.stop();

            case PINCH_GRIP:
               individualRobotiqHandController.pinchGrip();
               break;

            case BASIC_GRIP:
               individualRobotiqHandController.basicGrip();
               break;

            case WIDE_GRIP:
               individualRobotiqHandController.wideGrip();
               break;

            default:
               break;
            }
         }
      }
   }

   public void write()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (hasRobotiqHand.get(robotSide))
         {
            List<OneDegreeOfFreedomJoint> oneSideFingerJoints = allFingerJoints.get(robotSide);
            for (int i = 0; i < oneSideFingerJoints.size(); i++)
            {
               OneDegreeOfFreedomJoint joint = oneSideFingerJoints.get(i);
               joint.setKp(kpMap.get(joint).getDoubleValue());
               joint.setKd(kdMap.get(joint).getDoubleValue());
            }
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         if (hasRobotiqHand.get(robotSide))
            individualHandControllers.get(robotSide).writeDesiredJointAngles();
      }
   }

   @Override
   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   public SideDependentList<List<OneDegreeOfFreedomJoint>> getAllFingerJoints()
   {
      return allFingerJoints;
   }

   @Override
   protected void cleanup()
   {
      if (jointAngleProducer != null)
      {
         jointAngleProducer.cleanup();
      }
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData context)
   {
      write();
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData context)
   {
      timestamp = context.getTimestamp();
      read();
   }
}
