package us.ihmc.avatar.ros;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;

import org.apache.commons.lang3.tuple.ImmutableTriple;
import org.ros.message.Time;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosImuPublisher;
import us.ihmc.utilities.ros.publisher.RosInt32Publisher;
import us.ihmc.utilities.ros.publisher.RosJointStatePublisher;
import us.ihmc.utilities.ros.publisher.RosLastReceivedMessagePublisher;
import us.ihmc.utilities.ros.publisher.RosOdometryPublisher;
import us.ihmc.utilities.ros.publisher.RosStringPublisher;
import us.ihmc.utilities.ros.publisher.RosWrenchPublisher;

public class RosRobotConfigurationDataPublisher implements PacketConsumer<RobotConfigurationData>, Runnable
{
   public static final String JOINT_STATE_TOPIC = "/output/joint_states";
   public static final String WORLD_FRAME = "world";
   private final IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();

   private final RosTfPublisher tfPublisher;

   private final HashMap<RosJointStatePublisher, JointStatePublisherHelper> additionalJointStatePublisherMap = new HashMap<RosJointStatePublisher, JointStatePublisherHelper>();
   private RosJointStatePublisher[] additionalJointStatePublishers = new RosJointStatePublisher[0];
   private final RosJointStatePublisher jointStatePublisher;
   private final RosImuPublisher[] imuPublishers;
   private final RosOdometryPublisher pelvisOdometryPublisher;
   private final RosStringPublisher robotMotionStatusPublisher;
   private final RosInt32Publisher robotBehaviorPublisher;
   private final RosLastReceivedMessagePublisher lastReceivedMessagePublisher;
   private final ForceSensorDefinition[] forceSensorDefinitions;
   private final IMUDefinition[] imuDefinitions;
   private final ArrayList<String> nameList = new ArrayList<String>();
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final ArrayBlockingQueue<RobotConfigurationData> availableRobotConfigurationData = new ArrayBlockingQueue<RobotConfigurationData>(30);
   private final JointNameMap jointMap;
   private final int jointNameHash;

   private final SideDependentList<Integer> feetForceSensorIndexes = new SideDependentList<Integer>();
   private final SideDependentList<Integer> handForceSensorIndexes = new SideDependentList<Integer>();
   private final SideDependentList<RosWrenchPublisher> footForceSensorPublishers = new SideDependentList<RosWrenchPublisher>();
   private final SideDependentList<RosWrenchPublisher> wristForceSensorPublishers = new SideDependentList<RosWrenchPublisher>();
   private final SideDependentList<float[]> footForceSensorWrenches = new SideDependentList<float[]>();
   private final SideDependentList<float[]> wristForceSensorWrenches = new SideDependentList<float[]>();

   private final ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> staticTransforms;

   public RosRobotConfigurationDataPublisher(FullRobotModelFactory sdfFullRobotModelFactory, PacketCommunicator rosModulePacketCommunicator,
         final RosMainNode rosMainNode, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation,
         JointNameMap jointMap, String rosNameSpace, RosTfPublisher tfPublisher)
   {
      FullRobotModel fullRobotModel = sdfFullRobotModelFactory.createFullRobotModel();
      this.forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      this.imuDefinitions = fullRobotModel.getIMUDefinitions();
      this.rosMainNode = rosMainNode;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
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

      SideDependentList<String> feetForceSensorNames =  sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> handForceSensorNames =  sensorInformation.getWristForceSensorNames();

      for(RobotSide robotSide : RobotSide.values())
      {
         footForceSensorPublishers.put(robotSide, new RosWrenchPublisher(latched));
         feetForceSensorIndexes.put(robotSide, getForceSensorIndex(feetForceSensorNames.get(robotSide), forceSensorDefinitions));
         if(handForceSensorNames != null && !handForceSensorNames.isEmpty())
         {
            handForceSensorIndexes.put(robotSide, getForceSensorIndex(handForceSensorNames.get(robotSide), forceSensorDefinitions));
            wristForceSensorPublishers.put(robotSide, new RosWrenchPublisher(latched));
         }
      }

      OneDoFJoint[] joints = fullRobotModel.getControllableOneDoFJoints();
      for (int i = 0; i < joints.length; i++)
      {
         nameList.add(joints[i].getName());
      }

      jointNameHash = RobotConfigurationData.calculateJointNameHash(joints, forceSensorDefinitions, imuDefinitions);

      rosMainNode.attachPublisher(rosNameSpace + JOINT_STATE_TOPIC, jointStatePublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/robot_pose", pelvisOdometryPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/robot_motion_status", robotMotionStatusPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/behavior", robotBehaviorPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/foot_force_sensor/left", footForceSensorPublishers.get(RobotSide.LEFT));
      rosMainNode.attachPublisher(rosNameSpace + "/output/foot_force_sensor/right", footForceSensorPublishers.get(RobotSide.RIGHT));

      if(!wristForceSensorPublishers.isEmpty())
      {
         rosMainNode.attachPublisher(rosNameSpace + "/output/wrist_force_sensor/left", wristForceSensorPublishers.get(RobotSide.LEFT));
         rosMainNode.attachPublisher(rosNameSpace + "/output/wrist_force_sensor/right", wristForceSensorPublishers.get(RobotSide.RIGHT));
      }

      rosModulePacketCommunicator.attachListener(RobotConfigurationData.class, this);

      Thread t = new Thread(this, "RosRobotJointStatePublisher");
      t.start();
   }

   private int getForceSensorIndex(String forceSensorName, ForceSensorDefinition[] forceSensorDefinitions)
   {
      for(int i = 0; i < forceSensorDefinitions.length; i++)
      {
         if(forceSensorDefinitions[i].getSensorName().equals(forceSensorName))
         {
            return i;
         }
      }
      return -1;
   }

   @Override
   public void receivedPacket(RobotConfigurationData robotConfigurationData)
   {
      if(!availableRobotConfigurationData.offer(robotConfigurationData))
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
      while (true)
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
            float[] jointAngles = robotConfigurationData.getJointAngles();
            float[] jointVelocities = robotConfigurationData.getJointVelocities();
            float[] jointTorques = robotConfigurationData.getJointTorques();

            long timeStamp = ppsTimestampOffsetProvider.adjustRobotTimeStampToRosClock(robotConfigurationData.getTimestamp());
            Time t = Time.fromNano(timeStamp);

            if (robotConfigurationData.jointNameHash != jointNameHash)
            {
               throw new RuntimeException("Joint names do not match for RobotConfigurationData");
            }

            for (int i = 0; i < additionalJointStatePublishers.length; i++)
            {
               RosJointStatePublisher jointStatePublisher = additionalJointStatePublishers[i];
               JointStatePublisherHelper pubData = additionalJointStatePublisherMap.get(jointStatePublisher);
               pubData.publish(jointAngles, jointVelocities, jointTorques, t);
            }

            RigidBodyTransform pelvisTransform = new RigidBodyTransform(robotConfigurationData.getPelvisOrientation(), robotConfigurationData.getPelvisTranslation());

            jointStatePublisher.publish(nameList, jointAngles, jointVelocities, jointTorques, t);

            for (RobotSide robotSide : RobotSide.values())
            {
               footForceSensorWrenches.put(robotSide, robotConfigurationData.getMomentAndForceVectorForSensor(feetForceSensorIndexes.get(robotSide)));
               footForceSensorPublishers.get(robotSide).publish(timeStamp, footForceSensorWrenches.get(robotSide));

               if(!handForceSensorIndexes.isEmpty())
               {
                  wristForceSensorWrenches.put(robotSide, robotConfigurationData.getMomentAndForceVectorForSensor(handForceSensorIndexes.get(robotSide)));
                  wristForceSensorPublishers.get(robotSide).publish(timeStamp, wristForceSensorWrenches.get(robotSide));
               }
            }

            for (int sensorNumber = 0; sensorNumber < imuDefinitions.length; sensorNumber++)
            {
               RosImuPublisher rosImuPublisher = this.imuPublishers[sensorNumber];
               IMUPacket imuPacket = robotConfigurationData.getImuPacketForSensor(sensorNumber);
               ReferenceFrame imuFrame = imuDefinitions[sensorNumber].getIMUFrame();
               rosImuPublisher.publish(timeStamp, imuPacket, imuFrame.getName());
            }


            pelvisOdometryPublisher.publish(timeStamp, pelvisTransform, robotConfigurationData.getPelvisLinearVelocity(),
                  robotConfigurationData.getPelvisAngularVelocity(), jointMap.getUnsanitizedRootJointInSdf(), WORLD_FRAME);

            robotMotionStatusPublisher.publish(robotConfigurationData.getRobotMotionStatus().name());
            robotBehaviorPublisher.publish(robotConfigurationData.getRobotMotionStatus().getBehaviorId());

            tfPublisher.publish(pelvisTransform, timeStamp, WORLD_FRAME, jointMap.getUnsanitizedRootJointInSdf());
            if(staticTransforms != null)
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

            if(robotConfigurationData.getLastReceivedPacketTypeID() != -1)
            {
               Class<?> packetClass = netClassList.getClass(robotConfigurationData.getLastReceivedPacketTypeID());

               if(packetClass == null)
               {
                  System.err.println("Could not get packet class for ID " + robotConfigurationData.getLastReceivedPacketTypeID());
               }
               else
               {
                  String messageType = IHMCROSTranslationRuntimeTools.getROSMessageTypeStringFromIHMCMessageClass(packetClass);
                  if(messageType != null)
                  {
                     lastReceivedMessagePublisher.publish(messageType, robotConfigurationData.getLastReceivedPacketUniqueId(), robotConfigurationData.getTimestamp(), robotConfigurationData.getLastReceivedPacketRobotTimestamp());
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
               PrintTools.error(this, "DID NOT FIND JOINT " + jointNames[i]);
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
}
