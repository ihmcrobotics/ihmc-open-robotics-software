package us.ihmc.avatar.ros;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;

import org.apache.commons.lang3.tuple.ImmutableTriple;
import org.ros.message.Time;

import controller_msgs.msg.dds.IMUPacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
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

   private final RosJointStatePublisher jointStatePublisher;
   private final RosImuPublisher[] imuPublishers;
   private final RosOdometryPublisher pelvisOdometryPublisher;
   private final RosStringPublisher robotMotionStatusPublisher;
   private final RosInt32Publisher robotBehaviorPublisher;
   private final RosLastReceivedMessagePublisher lastReceivedMessagePublisher;
   private final ForceSensorDefinition[] forceSensorDefinitions;
   private final IMUDefinition[] imuDefinitions;
   private final int[] jointIndicesToIgnore;
   private final List<String> rosDefaultJointNames = new ArrayList<>();
   private final List<String> robotConfigurationDataJointNames = new ArrayList<>();
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

   private final ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> staticTransforms;

   private volatile boolean running = true;

   public RosRobotConfigurationDataPublisher(FullRobotModelFactory sdfFullRobotModelFactory, Ros2Node ros2Node, String robotConfigurationTopicName,
                                             final RosMainNode rosMainNode, RobotROSClockCalculator rosClockCalculator,
                                             AvatarRobotRosVisionSensorInformation sensorInformation, HumanoidForceSensorInformation forceSensorInformation,
                                             JointNameMap<?> jointMap, String rosNameSpace, RosTfPublisher tfPublisher)
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

      this.imuPublishers = new RosImuPublisher[imuDefinitions.length];
      for (int sensorNumber = 0; sensorNumber < imuDefinitions.length; sensorNumber++)
      {
         IMUDefinition imuDefinition = imuDefinitions[sensorNumber];
         String imuName = imuDefinition.getName();

         RosImuPublisher rosImuPublisher = new RosImuPublisher(latched);
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
      Set<String> jointsToIgnore = new HashSet<>();

      for (AvatarRobotLidarParameters lidarParameters : sensorInformation.getLidarParameters())
      {
         String lidarSpindleJointName = lidarParameters.getLidarSpindleJointName();
         jointsToIgnore.add(lidarSpindleJointName);
      }

      int index = 0;
      jointIndicesToIgnore = new int[jointsToIgnore.size()];
      Arrays.fill(jointIndicesToIgnore, -1);

      for (int i = 0; i < joints.length; i++)
      {
         robotConfigurationDataJointNames.add(joints[i].getName());
         if (jointsToIgnore.contains(joints[i].getName()))
            jointIndicesToIgnore[index++] = i;
      }

      rosDefaultJointNames.addAll(robotConfigurationDataJointNames);
      rosDefaultJointNames.removeAll(jointsToIgnore);

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

      ROS2Tools.createCallbackSubscription(ros2Node, RobotConfigurationData.class, robotConfigurationTopicName, s -> receivedPacket(s.takeNextData()));

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

         if (robotConfigurationData.getJointNameHash() != jointNameHash)
         {
            throw new RuntimeException("Joint names do not match for RobotConfigurationData");
         }

         if (rosMainNode.isStarted())
         {
            long rostimeStamp = rosClockCalculator.computeROSTime(robotConfigurationData.getWallTime(), robotConfigurationData.getMonotonicTime());
            Time rosTime = Time.fromNano(rostimeStamp);

            double[] jointAngles = toDoubleArray(robotConfigurationData.getJointAngles(), jointIndicesToIgnore);
            double[] jointVelocities = toDoubleArray(robotConfigurationData.getJointVelocities(), jointIndicesToIgnore);
            double[] jointTorques = toDoubleArray(robotConfigurationData.getJointTorques(), jointIndicesToIgnore);
            jointStatePublisher.publish(rosDefaultJointNames, jointAngles, jointVelocities, jointTorques, rosTime);

            RigidBodyTransform pelvisTransform = new RigidBodyTransform(robotConfigurationData.getRootOrientation(),
                                                                        robotConfigurationData.getRootTranslation());

            if (publishForceSensorInformation)
            {
               for (RobotSide robotSide : RobotSide.values())
               {
                  float[] arrayToPublish = new float[6];
                  robotConfigurationData.getForceSensorData().get(feetForceSensorIndexes.get(robotSide)).getAngularPart().get(0, arrayToPublish);
                  robotConfigurationData.getForceSensorData().get(feetForceSensorIndexes.get(robotSide)).getLinearPart().get(3, arrayToPublish);
                  footForceSensorWrenches.put(robotSide, arrayToPublish);
                  footForceSensorPublishers.get(robotSide).publish(rostimeStamp, footForceSensorWrenches.get(robotSide));

                  if (!handForceSensorIndexes.isEmpty())
                  {
                     arrayToPublish = new float[6];
                     robotConfigurationData.getForceSensorData().get(handForceSensorIndexes.get(robotSide)).getAngularPart().get(0, arrayToPublish);
                     robotConfigurationData.getForceSensorData().get(handForceSensorIndexes.get(robotSide)).getLinearPart().get(3, arrayToPublish);
                     wristForceSensorWrenches.put(robotSide, arrayToPublish);
                     wristForceSensorPublishers.get(robotSide).publish(rostimeStamp, wristForceSensorWrenches.get(robotSide));
                  }
               }
            }

            for (int sensorNumber = 0; sensorNumber < Math.min(imuDefinitions.length, robotConfigurationData.getImuSensorData().size()); sensorNumber++)
            {
               RosImuPublisher rosImuPublisher = imuPublishers[sensorNumber];
               IMUPacket imuPacket = robotConfigurationData.getImuSensorData().get(sensorNumber);
               ReferenceFrame imuFrame = imuDefinitions[sensorNumber].getIMUFrame();
               rosImuPublisher.publish(rostimeStamp, imuPacket, imuFrame.getName());
            }

            pelvisOdometryPublisher.publish(rostimeStamp,
                                            pelvisTransform,
                                            robotConfigurationData.getPelvisLinearVelocity(),
                                            robotConfigurationData.getPelvisAngularVelocity(),
                                            jointMap.getUnsanitizedRootJointInSdf(),
                                            WORLD_FRAME);

            robotMotionStatusPublisher.publish(RobotMotionStatus.fromByte(robotConfigurationData.getRobotMotionStatus()).name());
            robotBehaviorPublisher.publish(RobotMotionStatus.fromByte(robotConfigurationData.getRobotMotionStatus()).getBehaviorId());

            tfPublisher.publish(pelvisTransform, rostimeStamp, WORLD_FRAME, jointMap.getUnsanitizedRootJointInSdf());
            if (staticTransforms != null)
            {
               for (int i = 0; i < staticTransforms.size(); i++)
               {
                  ImmutableTriple<String, String, RigidBodyTransform> staticTransformTriplet = staticTransforms.get(i);
                  String from = staticTransformTriplet.getLeft();
                  String to = staticTransformTriplet.getMiddle();
                  RigidBodyTransform staticTransform = staticTransformTriplet.getRight();
                  tfPublisher.publish(staticTransform, rostimeStamp, from, to);
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
                                                          rostimeStamp,
                                                          robotConfigurationData.getLastReceivedPacketRobotTimestamp());
                  }
               }
            }
         }
      }
   }

   private static double[] toDoubleArray(TFloatArrayList input, int[] indicesToIgnore)
   {
      double[] output = new double[input.size() - indicesToIgnore.length];
      int ignoreIndex = 0;
      int outputIndex = 0;

      for (int inputIndex = 0; inputIndex < input.size(); inputIndex++)
      {
         if (ignoreIndex < indicesToIgnore.length && indicesToIgnore[ignoreIndex] >= 0 && inputIndex == indicesToIgnore[ignoreIndex])
         {
            ignoreIndex++;
         }
         else
         {
            output[outputIndex++] = input.get(inputIndex);
         }
      }

      return output;
   }

   @Override
   public void closeAndDispose()
   {
      running = false;
   }
}
