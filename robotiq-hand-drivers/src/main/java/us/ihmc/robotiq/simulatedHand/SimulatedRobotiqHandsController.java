package us.ihmc.robotiq.simulatedHand;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotiq.model.RobotiqHandModel;
import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulatedRobotiqHandsController implements RobotController
{
   private final boolean DEBUG = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoBoolean sendFingerJointGains;

   private final LinkedHashMap<OneDoFJointBasics, YoDouble> kpMap = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJointBasics, YoDouble> kdMap = new LinkedHashMap<>();

   private final YoDouble fingerTrajectoryTime;

   private final SideDependentList<HandDesiredConfigurationMessageSubscriber> handDesiredConfigurationMessageSubscribers = new SideDependentList<>();

   private final SideDependentList<List<OneDoFJointBasics>> allFingerJoints = new SideDependentList<>();

   private final RobotiqHandModel handModel = new RobotiqHandModel();

   private final SideDependentList<IndividualRobotiqHandController> individualHandControllers = new SideDependentList<>();

   private final SimulatedRobotiqHandJointAngleProducer jointAngleProducer;

   private final SideDependentList<Boolean> hasRobotiqHand = new SideDependentList<>(false, false);

   // TODO These 2 joints don't exist on the real robot but are somehow present in the SDF. Need to keep them in place or they will drift.
   private final SideDependentList<OneDoFJointBasics> palmMiddleFingerJoints = new SideDependentList<>();
   private final YoDouble kpPalmFingerMiddleJoint = new YoDouble("kpPalmMiddleFingerJoint", registry);
   private final YoDouble kdPalmFingerMiddleJoint = new YoDouble("kdPalmMiddleFingerJoint", registry);

   private final FullRobotModel fullRobotModel;
   private final JointDesiredOutputListBasics jointDesiredOutputList;
   private RobotSide[] sides;
   private final JointDesiredControlMode jointDesiredControlMode;

   public SimulatedRobotiqHandsController(FullRobotModel fullRobotModel,
                                          JointDesiredOutputListBasics jointDesiredOutputList,
                                          DoubleProvider yoTime,
                                          RealtimeROS2Node realtimeROS2Node,
                                          ROS2Topic<?> outputTopic,
                                          ROS2Topic<?> inputTopic,
                                          RobotSide[] sides)
   {
      this(fullRobotModel, jointDesiredOutputList, yoTime, realtimeROS2Node, outputTopic, inputTopic, sides, JointDesiredControlMode.EFFORT);
   }

   public SimulatedRobotiqHandsController(FullRobotModel fullRobotModel,
                                          JointDesiredOutputListBasics jointDesiredOutputList,
                                          DoubleProvider yoTime,
                                          RealtimeROS2Node realtimeROS2Node,
                                          ROS2Topic<?> outputTopic,
                                          ROS2Topic<?> inputTopic,
                                          RobotSide[] sides,
                                          JointDesiredControlMode jointDesiredControlMode)
   {
      this.fullRobotModel = fullRobotModel;
      this.jointDesiredOutputList = jointDesiredOutputList;
      this.sides = sides;
      this.jointDesiredControlMode = jointDesiredControlMode;
      sendFingerJointGains = new YoBoolean("sendFingerJointGains", registry);
      fingerTrajectoryTime = new YoDouble("FingerTrajectoryTime", registry);

      sendFingerJointGains.set(true);

      if (realtimeROS2Node != null)
      {
         ROS2PublisherBasics<HandJointAnglePacket> jointAnglePublisher = realtimeROS2Node.createPublisher(outputTopic.withTypeName(HandJointAnglePacket.class));
         jointAngleProducer = new SimulatedRobotiqHandJointAngleProducer(jointAnglePublisher, fullRobotModel);
      }
      else
      {
         jointAngleProducer = null;
      }
      fingerTrajectoryTime.set(0.5);

      EnumMap<RobotiqHandJointNameMinimal, YoDouble> kpEnumMap = new EnumMap<>(RobotiqHandJointNameMinimal.class);
      EnumMap<RobotiqHandJointNameMinimal, YoDouble> kdEnumMap = new EnumMap<>(RobotiqHandJointNameMinimal.class);

      setupGains(kpEnumMap, kdEnumMap, registry);

      for (RobotSide robotSide : sides)
      {
         palmMiddleFingerJoints.put(robotSide, fullRobotModel.getOneDoFJointByName(RobotiqHandModel.getPalmFingerMiddleJointName(robotSide)));
         allFingerJoints.put(robotSide, new ArrayList<>());

         for (HandJointName jointEnum : handModel.getHandJointNames())
         {
            OneDoFJointBasics fingerJoint = fullRobotModel.getOneDoFJointByName(jointEnum.getJointName(robotSide));
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
            if (realtimeROS2Node != null)
            {
               ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node,
                                                             HandDesiredConfigurationMessage.class,
                                                             inputTopic,
                                                             handDesiredConfigurationSubscriber);
            }

            IndividualRobotiqHandController individualHandController = new IndividualRobotiqHandController(robotSide,
                                                                                                           yoTime,
                                                                                                           fingerTrajectoryTime,
                                                                                                           fullRobotModel,
                                                                                                           registry);
            individualHandControllers.put(robotSide, individualHandController);
         }
      }
   }

   private void setupGains(EnumMap<RobotiqHandJointNameMinimal, YoDouble> kpEnumMap,
                           EnumMap<RobotiqHandJointNameMinimal, YoDouble> kdEnumMap,
                           YoRegistry registry)
   {
      YoDouble kpFingerJoint1 = new YoDouble("kpFingerJoint1", registry);
      YoDouble kpFingerJoint2 = new YoDouble("kpFingerJoint2", registry);
      YoDouble kpFingerJoint3 = new YoDouble("kpFingerJoint3", registry);
      YoDouble kpThumbJoint1 = new YoDouble("kpThumbJoint1", registry);
      YoDouble kpThumbJoint2 = new YoDouble("kpThumbJoint2", registry);
      YoDouble kpThumbJoint3 = new YoDouble("kpThumbJoint3", registry);

      kpPalmFingerMiddleJoint.set(20.0);

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

      kdPalmFingerMiddleJoint.set(1.0);

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
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      if (jointAngleProducer != null)
      {
         jointAngleProducer.sendHandJointAnglesPacket();
      }

      checkForNewHandDesiredConfigurationRequested();

      for (RobotSide robotSide : sides)
      {
         if (hasRobotiqHand.get(robotSide))
            individualHandControllers.get(robotSide).doControl();
      }

      for (RobotSide robotSide : sides)
      {
         if (hasRobotiqHand.get(robotSide))
         {
            List<OneDoFJointBasics> oneSideFingerJoints = allFingerJoints.get(robotSide);

            for (OneDoFJointBasics joint : oneSideFingerJoints)
            {
               JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
               jointDesiredOutput.setControlMode(jointDesiredControlMode);
               jointDesiredOutput.setStiffness(kpMap.get(joint).getDoubleValue());
               jointDesiredOutput.setDamping(kdMap.get(joint).getDoubleValue());
            }

            JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(palmMiddleFingerJoints.get(robotSide));
            jointDesiredOutput.setControlMode(jointDesiredControlMode);
            jointDesiredOutput.setDesiredPosition(0.0);
            jointDesiredOutput.setDesiredVelocity(0.0);
            jointDesiredOutput.setStiffness(kpPalmFingerMiddleJoint.getValue());
            jointDesiredOutput.setDamping(kdPalmFingerMiddleJoint.getValue());
         }

         if (hasRobotiqHand.get(robotSide))
            individualHandControllers.get(robotSide).writeDesiredJointAngles(jointDesiredOutputList);
      }
   }

   private void checkForNewHandDesiredConfigurationRequested()
   {
      for (RobotSide robotSide : sides)
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

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public SideDependentList<List<OneDoFJointBasics>> getAllFingerJoints()
   {
      return allFingerJoints;
   }

   protected void cleanup()
   {
      if (jointAngleProducer != null)
      {
         jointAngleProducer.cleanup();
      }
   }
}
