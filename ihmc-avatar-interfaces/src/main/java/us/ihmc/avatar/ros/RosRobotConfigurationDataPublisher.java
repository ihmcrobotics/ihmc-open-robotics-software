package us.ihmc.avatar.ros;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;

import org.apache.commons.lang3.tuple.ImmutableTriple;
import org.ros.message.Time;

import controller_msgs.msg.dds.IMUPacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.parameters.AvatarRobotRosVisionSensorInformation;
import us.ihmc.sensorProcessing.parameters.HumanoidForceSensorInformation;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosImuPublisher;
import us.ihmc.utilities.ros.publisher.RosInt32Publisher;
import us.ihmc.utilities.ros.publisher.RosJointStatePublisher;
import us.ihmc.utilities.ros.publisher.RosLastReceivedMessagePublisher;
import us.ihmc.utilities.ros.publisher.RosOdometryPublisher;
import us.ihmc.utilities.ros.publisher.RosStringPublisher;
import us.ihmc.utilities.ros.publisher.RosWrenchPublisher;

public class RosRobotConfigurationDataPublisher implements PacketConsumer<RobotConfigurationData>, Runnable, CloseableAndDisposable
{
   public static final String JOINT_STATE_TOPIC = "/output/joint_states";
   public static final String WORLD_FRAME = "world";
   private final IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();

   private final RosTfPublisher tfPublisher;

   private final HashMap<RosJointStatePublisher, JointStatePublisherHelper> additionalJointStatePublisherMap = new HashMap<RosJointStatePublisher, JointStatePublisherHelper>();
   private RosJointStatePublisher[] additionalJointStatePublishers = new RosJointStatePublisher[0];
   private final RosJointStatePublisher jointStatePublisher;
   private final String[] imuROSFrameIDs;
   private final RosImuPublisher[] imuPublishers;
   private final RosOdometryPublisher pelvisOdometryPublisher;
   private final RosStringPublisher robotMotionStatusPublisher;
   private final RosInt32Publisher robotBehaviorPublisher;
   private final RosLastReceivedMessagePublisher lastReceivedMessagePublisher;
   private final ForceSensorDefinition[] forceSensorDefinitions;
   private final IMUDefinition[] imuDefinitions;
   private final ArrayList<String> nameList = new ArrayList<String>();
   private final RosMainNode rosMainNode;
   private final RobotROSClockCalculator rosClockCalculator;
   private final ArrayBlockingQueue<RobotConfigurationData> availableRobotConfigurationData = new ArrayBlockingQueue<RobotConfigurationData>(30);
   private final JointNameMap<?> jointMap;
   private final int jointNameHash;

   private boolean publishForceSensorInformation = false;
   private final SideDependentList<Integer> feetForceSensorIndexes = new SideDependentList<Integer>();
   private final SideDependentList<Integer> handForceSensorIndexes = new SideDependentList<Integer>();
   private final SideDependentList<RosWrenchPublisher> footForceSensorPublishers = new SideDependentList<RosWrenchPublisher>();
   private final SideDependentList<RosWrenchPublisher> wristForceSensorPublishers = new SideDependentList<RosWrenchPublisher>();
   private final SideDependentList<float[]> footForceSensorWrenches = new SideDependentList<float[]>();
   private final SideDependentList<float[]> wristForceSensorWrenches = new SideDependentList<float[]>();

   private final List<ImmutableTriple<String, String, RigidBodyTransform>> staticTransforms;

   private volatile boolean running = true;

   public RosRobotConfigurationDataPublisher(FullRobotModelFactory sdfFullRobotModelFactory,
                                             RealtimeROS2Node ros2Node,
                                             ROS2Topic<?> robotConfigurationTopicName,
                                             final RosMainNode rosMainNode,
                                             RobotROSClockCalculator rosClockCalculator,
                                             AvatarRobotRosVisionSensorInformation sensorInformation,
                                             HumanoidForceSensorInformation forceSensorInformation,
                                             JointNameMap<?> jointMap,
                                             String rosNameSpace,
                                             RosTfPublisher tfPublisher)
   {
      FullRobotModel fullRobotModel = sdfFullRobotModelFactory.createFullRobotModel();
      this.forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      this.imuDefinitions = fullRobotModel.getIMUDefinitions();
      this.rosMainNode = rosMainNode;
      this.rosClockCalculator = rosClockCalculator;
      this.tfPublisher = tfPublisher;
      this.jointMap = jointMap;
      this.staticTransforms = sensorInformation.getStaticTransformsForRos();

      boolean latched = false;
      this.jointStatePublisher = new RosJointStatePublisher(latched);
      this.pelvisOdometryPublisher = new RosOdometryPublisher(latched);
      this.robotMotionStatusPublisher = new RosStringPublisher(latched);
      this.robotBehaviorPublisher = new RosInt32Publisher(latched);
      this.lastReceivedMessagePublisher = new RosLastReceivedMessagePublisher(latched);

      this.imuROSFrameIDs = new String[imuDefinitions.length];
      this.imuPublishers = new RosImuPublisher[imuDefinitions.length];

      for (int sensorNumber = 0; sensorNumber < imuDefinitions.length; sensorNumber++)
      {
         IMUDefinition imuDefinition = imuDefinitions[sensorNumber];
         String imuName = imuDefinition.getName();

         RosImuPublisher rosImuPublisher = new RosImuPublisher(latched);
         this.imuROSFrameIDs[sensorNumber] = imuDefinition.getName() + "_Frame";
         this.imuPublishers[sensorNumber] = rosImuPublisher;
         rosMainNode.attachPublisher(rosNameSpace + "/output/imu/" + imuName, rosImuPublisher);
      }

      if (forceSensorInformation != null)
      {
         publishForceSensorInformation = true;
         SideDependentList<String> feetForceSensorNames = forceSensorInformation.getFeetForceSensorNames();
         SideDependentList<String> handForceSensorNames = forceSensorInformation.getWristForceSensorNames();

         for (RobotSide robotSide : RobotSide.values())
         {
            footForceSensorPublishers.put(robotSide, new RosWrenchPublisher(latched));
            feetForceSensorIndexes.put(robotSide, getForceSensorIndex(feetForceSensorNames.get(robotSide), forceSensorDefinitions));
            if (handForceSensorNames != null && !handForceSensorNames.isEmpty())
            {
               handForceSensorIndexes.put(robotSide, getForceSensorIndex(handForceSensorNames.get(robotSide), forceSensorDefinitions));
               wristForceSensorPublishers.put(robotSide, new RosWrenchPublisher(latched));
            }
         }
      }

      OneDoFJointBasics[] joints = fullRobotModel.getControllableOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         nameList.add(joints[i].getName());
      }

      jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(joints, forceSensorDefinitions, imuDefinitions);

      rosMainNode.attachPublisher(rosNameSpace + JOINT_STATE_TOPIC, jointStatePublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/robot_pose", pelvisOdometryPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/robot_motion_status", robotMotionStatusPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/behavior", robotBehaviorPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/last_robot_config_received", lastReceivedMessagePublisher);

      if (publishForceSensorInformation)
      {
         rosMainNode.attachPublisher(rosNameSpace + "/output/foot_force_sensor/left", footForceSensorPublishers.get(RobotSide.LEFT));
         rosMainNode.attachPublisher(rosNameSpace + "/output/foot_force_sensor/right", footForceSensorPublishers.get(RobotSide.RIGHT));
         if (!wristForceSensorPublishers.isEmpty())
         {
            rosMainNode.attachPublisher(rosNameSpace + "/output/wrist_force_sensor/left", wristForceSensorPublishers.get(RobotSide.LEFT));
            rosMainNode.attachPublisher(rosNameSpace + "/output/wrist_force_sensor/right", wristForceSensorPublishers.get(RobotSide.RIGHT));
         }
      }

      ros2Node.createSubscription(robotConfigurationTopicName.withTypeName(RobotConfigurationData.class), s -> receivedPacket(s.takeNextData()));

      Thread t = new Thread(this, "RosRobotJointStatePublisher");
      t.start();
   }

   private int getForceSensorIndex(String forceSensorName, ForceSensorDefinition[] forceSensorDefinitions)
   {
      for (int i = 0; i < forceSensorDefinitions.length; i++)
      {
         if (forceSensorDefinitions[i].getSensorName().equals(forceSensorName))
         {
            return i;
         }
      }
      return -1;
   }

   @Override
   public void receivedPacket(RobotConfigurationData robotConfigurationData)
   {
      if (!availableRobotConfigurationData.offer(robotConfigurationData))
      {
         availableRobotConfigurationData.clear();
      }
   }

   public void setAdditionalJointStatePublishing(String topicName, String... jointNames)
   {
      RosJointStatePublisher jointStatePublisher = new RosJointStatePublisher(false);
      rosMainNode.attachPublisher(topicName, jointStatePublisher);

      JointStatePublisherHelper pubData = new JointStatePublisherHelper(jointStatePublisher, jointNames);
      additionalJointStatePublisherMap.put(jointStatePublisher, pubData);
      rebuildKeySetArray();
   }

   private void rebuildKeySetArray()
   {
      Set<RosJointStatePublisher> keySet = additionalJointStatePublisherMap.keySet();
      additionalJointStatePublishers = new RosJointStatePublisher[keySet.size()];
      additionalJointStatePublishers = keySet.toArray(additionalJointStatePublishers);
   }

   @Override
   public void run()
   {
      while (running)
      {
         RobotConfigurationData robotConfigurationData;
         try
         {
            robotConfigurationData = availableRobotConfigurationData.take();
         }
         catch (InterruptedException e)
         {
            // Ignore and skip to the next loop iteration
            continue;
         }
         if (rosMainNode.isStarted())
         {
            float[] jointAngles = robotConfigurationData.getJointAngles().toArray();
            float[] jointVelocities = robotConfigurationData.getJointVelocities().toArray();
            float[] jointTorques = robotConfigurationData.getJointTorques().toArray();

            long timeStamp = rosClockCalculator.computeROSTime(robotConfigurationData.getWallTime(), robotConfigurationData.getMonotonicTime());
            Time t = Time.fromNano(timeStamp);

            if (robotConfigurationData.getJointNameHash() != jointNameHash)
            {
               throw new RuntimeException("Joint names do not match for RobotConfigurationData");
            }

            for (int i = 0; i < additionalJointStatePublishers.length; i++)
            {
               RosJointStatePublisher jointStatePublisher = additionalJointStatePublishers[i];
               JointStatePublisherHelper pubData = additionalJointStatePublisherMap.get(jointStatePublisher);
               pubData.publish(jointAngles, jointVelocities, jointTorques, t);
            }

            RigidBodyTransform pelvisTransform = new RigidBodyTransform(robotConfigurationData.getRootOrientation(),
                                                                        robotConfigurationData.getRootPosition());

            jointStatePublisher.publish(nameList, jointAngles, jointVelocities, jointTorques, t);

            if (publishForceSensorInformation)
            {
               for (RobotSide robotSide : RobotSide.values())
               {
                  float[] arrayToPublish = new float[6];
                  robotConfigurationData.getForceSensorData().get(feetForceSensorIndexes.get(robotSide)).getAngularPart().get(0, arrayToPublish);
                  robotConfigurationData.getForceSensorData().get(feetForceSensorIndexes.get(robotSide)).getLinearPart().get(3, arrayToPublish);
                  footForceSensorWrenches.put(robotSide, arrayToPublish);
                  footForceSensorPublishers.get(robotSide).publish(timeStamp, footForceSensorWrenches.get(robotSide));

                  if (!handForceSensorIndexes.isEmpty())
                  {
                     arrayToPublish = new float[6];
                     robotConfigurationData.getForceSensorData().get(handForceSensorIndexes.get(robotSide)).getAngularPart().get(0, arrayToPublish);
                     robotConfigurationData.getForceSensorData().get(handForceSensorIndexes.get(robotSide)).getLinearPart().get(3, arrayToPublish);
                     wristForceSensorWrenches.put(robotSide, arrayToPublish);
                     wristForceSensorPublishers.get(robotSide).publish(timeStamp, wristForceSensorWrenches.get(robotSide));
                  }
               }
            }

            for (int sensorNumber = 0; sensorNumber < Math.min(imuDefinitions.length, robotConfigurationData.getImuSensorData().size()); sensorNumber++)
            {
               RosImuPublisher rosImuPublisher = this.imuPublishers[sensorNumber];
               IMUPacket imuPacket = robotConfigurationData.getImuSensorData().get(sensorNumber);
               rosImuPublisher.publish(timeStamp, imuPacket, imuROSFrameIDs[sensorNumber]);
            }

            pelvisOdometryPublisher.publish(timeStamp,
                                            pelvisTransform,
                                            robotConfigurationData.getPelvisLinearVelocity(),
                                            robotConfigurationData.getPelvisAngularVelocity(),
                                            jointMap.getUnsanitizedRootJointInSdf(),
                                            WORLD_FRAME);

            robotMotionStatusPublisher.publish(RobotMotionStatus.fromByte(robotConfigurationData.getRobotMotionStatus()).name());
            robotBehaviorPublisher.publish(RobotMotionStatus.fromByte(robotConfigurationData.getRobotMotionStatus()).getBehaviorId());

            tfPublisher.publish(pelvisTransform, timeStamp, WORLD_FRAME, jointMap.getUnsanitizedRootJointInSdf());
            if (staticTransforms != null)
            {
               for (int i = 0; i < staticTransforms.size(); i++)
               {
                  ImmutableTriple<String, String, RigidBodyTransform> staticTransformTriplet = staticTransforms.get(i);
                  String from = staticTransformTriplet.getLeft();
                  String to = staticTransformTriplet.getMiddle();
                  RigidBodyTransform staticTransform = staticTransformTriplet.getRight();
                  tfPublisher.publish(staticTransform, timeStamp, from, to);
               }
            }

            if (robotConfigurationData.getLastReceivedPacketTypeId() != -1)
            {
               Class<?> packetClass = netClassList.getClass(robotConfigurationData.getLastReceivedPacketTypeId());

               if (packetClass == null)
               {
                  System.err.println("Could not get packet class for ID " + robotConfigurationData.getLastReceivedPacketTypeId());
               }
               else
               {
                  String messageType = IHMCROSTranslationRuntimeTools.getROSMessageTypeStringFromIHMCMessageClass(packetClass);
                  if (messageType != null)
                  {
                     lastReceivedMessagePublisher.publish(messageType,
                                                          robotConfigurationData.getLastReceivedPacketUniqueId(),
                                                          timeStamp,
                                                          robotConfigurationData.getLastReceivedPacketRobotTimestamp());
                  }
               }
            }
         }
      }
   }

   private class JointStatePublisherHelper
   {
      private final RosJointStatePublisher jointStatePublisher;
      private final ArrayList<String> jointNames = new ArrayList<String>();
      private final int[] jointIndices;
      private final double[] jointAnglesSubSet;
      private final double[] jointVelocitiesSubSet;
      private final double[] jointJointTorquesSubSet;

      public JointStatePublisherHelper(RosJointStatePublisher jointStatePublisher, String[] jointNames)
      {
         this.jointStatePublisher = jointStatePublisher;
         jointIndices = new int[jointNames.length];
         for (int i = 0; i < jointNames.length; i++)
         {
            int index = nameList.indexOf(jointNames[i]);
            if (index == -1)
            {
               LogTools.error("DID NOT FIND JOINT " + jointNames[i]);
               continue;
            }
            this.jointNames.add(jointNames[i]);
            jointIndices[i] = index;
         }

         jointAnglesSubSet = new double[this.jointNames.size()];
         jointVelocitiesSubSet = new double[this.jointNames.size()];
         jointJointTorquesSubSet = new double[this.jointNames.size()];
      }

      public void publish(float[] jointAngles, float[] jointVelocities, float[] jointTorques, Time t)
      {
         for (int i = 0; i < jointIndices.length; i++)
         {
            jointAnglesSubSet[i] = jointAngles[jointIndices[i]];
            jointVelocitiesSubSet[i] = jointVelocities[jointIndices[i]];
            jointJointTorquesSubSet[i] = jointTorques[jointIndices[i]];
         }
         jointStatePublisher.publish(jointNames, jointAnglesSubSet, jointVelocitiesSubSet, jointJointTorquesSubSet, t);
      }
   }

   @Override
   public void closeAndDispose()
   {
      running = false;
   }
}
