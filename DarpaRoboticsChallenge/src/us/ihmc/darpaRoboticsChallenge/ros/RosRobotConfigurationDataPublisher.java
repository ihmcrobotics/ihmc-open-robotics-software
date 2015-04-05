package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.ejml.data.DenseMatrix64F;
import org.ros.message.Time;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDefinition;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModelUtils;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.msgToPacket.IHMCRosApiMessageMap;
import us.ihmc.utilities.ros.publisher.RosCachedRawIMUDataPublisher;
import us.ihmc.utilities.ros.publisher.RosImuPublisher;
import us.ihmc.utilities.ros.publisher.RosInt32Publisher;
import us.ihmc.utilities.ros.publisher.RosJointStatePublisher;
import us.ihmc.utilities.ros.publisher.RosOdometryPublisher;
import us.ihmc.utilities.ros.publisher.RosStringPublisher;
import us.ihmc.utilities.ros.publisher.RosTrooperFootSensorPublisher;
import us.ihmc.utilities.ros.publisher.RosWrenchPublisher;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class RosRobotConfigurationDataPublisher implements PacketConsumer<RobotConfigurationData>, Runnable
{
   public static final String WORLD_FRAME = "world";

   private final RosTfPublisher tfPublisher;

   private final HashMap<RosJointStatePublisher, JointStatePublisherHelper> additionalJointStatePublisherMap = new HashMap<RosJointStatePublisher, JointStatePublisherHelper>();
   private RosJointStatePublisher[] additionalJointStatePublishers = new RosJointStatePublisher[0];
   private final RosJointStatePublisher jointStatePublisher;
   private final RosImuPublisher pelvisImuPublisher;
   private final RosCachedRawIMUDataPublisher batchImuPublisher;
   private final RosOdometryPublisher pelvisOdometryPublisher;
   private final RosStringPublisher robotMotionStatusPublisher;
   private final RosInt32Publisher robotBehaviorPublisher;
   private final ForceSensorDefinition[] forceSensorDefinitions;
   private final ArrayList<String> nameList = new ArrayList<String>();
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final ConcurrentLinkedQueue<RobotConfigurationData> availableRobotConfigurationData = new ConcurrentLinkedQueue<RobotConfigurationData>();
   private final RigidBodyTransform transformFromHeadToMultisenseHeadRoot = new RigidBodyTransform();
   private final int jointNameHash;
   
   private final SideDependentList<Integer> feetForceSensorIndexes = new SideDependentList<Integer>();
   private final SideDependentList<Integer> handForceSensorIndexes = new SideDependentList<Integer>();
   private final SideDependentList<RosWrenchPublisher> footForceSensorPublishers = new SideDependentList<RosWrenchPublisher>();
   private final SideDependentList<RosWrenchPublisher> wristForceSensorPublishers = new SideDependentList<RosWrenchPublisher>();
   private final SideDependentList<DenseMatrix64F> footForceSensorWrenches = new SideDependentList<DenseMatrix64F>();
   private final SideDependentList<DenseMatrix64F> wristForceSensorWrenches = new SideDependentList<DenseMatrix64F>();
   private final RosTrooperFootSensorPublisher bothFeetForceSensorPublisher;

   public RosRobotConfigurationDataPublisher(SDFFullRobotModelFactory sdfFullRobotModelFactory, PacketCommunicator rosModulePacketCommunicator,
         final RosMainNode rosMainNode, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation,
         String rosNameSpace, RosTfPublisher tfPublisher)
   {
      SDFFullRobotModel fullRobotModel = sdfFullRobotModelFactory.createFullRobotModel();
      this.forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      this.rosMainNode = rosMainNode;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.tfPublisher = tfPublisher;

      boolean latched = false;
      this.jointStatePublisher = new RosJointStatePublisher(latched);
      this.pelvisImuPublisher = new RosImuPublisher(latched);
      this.bothFeetForceSensorPublisher = new RosTrooperFootSensorPublisher(latched);
      this.batchImuPublisher = new RosCachedRawIMUDataPublisher(latched);
      this.pelvisOdometryPublisher = new RosOdometryPublisher(latched);
      this.robotMotionStatusPublisher = new RosStringPublisher(latched);
      this.robotBehaviorPublisher = new RosInt32Publisher(latched);
      
      SideDependentList<String> feetForceSensorNames =  sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> handForceSensorNames =  sensorInformation.getWristForceSensorNames();
      
      for(RobotSide robotSide : RobotSide.values())
      {
         footForceSensorPublishers.put(robotSide, new RosWrenchPublisher(latched));
         wristForceSensorPublishers.put(robotSide, new RosWrenchPublisher(latched));
         feetForceSensorIndexes.put(robotSide, getForceSensorIndex(feetForceSensorNames.get(robotSide), forceSensorDefinitions));
         handForceSensorIndexes.put(robotSide, getForceSensorIndex(handForceSensorNames.get(robotSide), forceSensorDefinitions));
      }

      OneDoFJoint[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      for (int i = 0; i < joints.length; i++)
      {
         nameList.add(joints[i].getName());
      }

      jointNameHash = RobotConfigurationData.calculateJointNameHash(joints, fullRobotModel.getForceSensorDefinitions());

      rosMainNode.attachPublisher(rosNameSpace + IHMCRosApiMessageMap.PACKET_TO_TOPIC_MAP.get(RobotConfigurationData.class), jointStatePublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/pelvisImu", pelvisImuPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/batch_raw_imu", batchImuPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/robot_pose", pelvisOdometryPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/robot_motion_status", robotMotionStatusPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/behavior", robotBehaviorPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/foot_force_sensor/both", bothFeetForceSensorPublisher);
      rosMainNode.attachPublisher(rosNameSpace + "/output/foot_force_sensor/left", footForceSensorPublishers.get(RobotSide.LEFT));
      rosMainNode.attachPublisher(rosNameSpace + "/output/foot_force_sensor/right", footForceSensorPublishers.get(RobotSide.RIGHT));
      rosMainNode.attachPublisher(rosNameSpace + "/output/wrist_force_sensor/left", wristForceSensorPublishers.get(RobotSide.LEFT));
      rosMainNode.attachPublisher(rosNameSpace + "/output/wrist_force_sensor/right", wristForceSensorPublishers.get(RobotSide.RIGHT));
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
      availableRobotConfigurationData.add(robotConfigurationData);
      if (availableRobotConfigurationData.size() > 30)
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
         RobotConfigurationData robotConfigurationData = availableRobotConfigurationData.poll();
         if (robotConfigurationData != null)
         {
            if (rosMainNode.isStarted())
            {
               double[] jointAngles = robotConfigurationData.getJointAngles();
               double[] jointVelocities = robotConfigurationData.getJointVelocities();
               double[] jointTorques = robotConfigurationData.getJointTorques();

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

               RigidBodyTransform pelvisTransform = new RigidBodyTransform(robotConfigurationData.getOrientation(), robotConfigurationData.getTranslation());

               jointStatePublisher.publish(nameList, jointAngles, jointVelocities, jointTorques, t);

               for(RobotSide robotSide : RobotSide.values())
               {
                  footForceSensorWrenches.put(robotSide, robotConfigurationData.getMomentAndForceVectorForSensor(feetForceSensorIndexes.get(robotSide)));
                  footForceSensorPublishers.get(robotSide).publish(timeStamp, footForceSensorWrenches.get(robotSide));
                  
                  wristForceSensorWrenches.put(robotSide, robotConfigurationData.getMomentAndForceVectorForSensor(handForceSensorIndexes.get(robotSide)));
                  wristForceSensorPublishers.get(robotSide).publish(timeStamp, wristForceSensorWrenches.get(robotSide));
               }
               
               bothFeetForceSensorPublisher.publish(timeStamp, footForceSensorWrenches.get(RobotSide.LEFT), footForceSensorWrenches.get(RobotSide.RIGHT));
               
               pelvisImuPublisher.publish(timeStamp, robotConfigurationData.getRawImuLinearAcceleration(), robotConfigurationData.getRawImuOrientation(),
                     robotConfigurationData.getRawImuAngularVelocity());
               batchImuPublisher.appendRawImuData(timeStamp, robotConfigurationData.getRawImuOrientation(),
                     robotConfigurationData.getRawImuLinearAcceleration());
               pelvisOdometryPublisher.publish(timeStamp, pelvisTransform, robotConfigurationData.getPelvisLinearVelocity(),
                     robotConfigurationData.getRawImuLinearAcceleration(), "/pelvis_imu");

               robotMotionStatusPublisher.publish(robotConfigurationData.getRobotMotionStatus().name());
               robotBehaviorPublisher.publish(robotConfigurationData.getRobotMotionStatus().getBehaviorId());

               tfPublisher.publish(pelvisTransform, timeStamp, WORLD_FRAME, "pelvis");
               tfPublisher.publish(transformFromHeadToMultisenseHeadRoot, timeStamp, "head", "multisense/head_root");
            }
         }
         ThreadTools.sleep(1);
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

      public void publish(double[] jointAngles, double[] jointVelocities, double[] jointTorques, Time t)
      {
         for (int i = 0; i < jointIndices.length; i++)
         {
            jointAnglesSubSet[i] = jointAngles[jointIndices[i]];
            jointVelocitiesSubSet[i] = jointVelocities[jointIndices[i]];
            jointJointTorquesSubSet[i] = jointTorques[jointIndices[i]];
         }
         jointStatePublisher.publish(nameList, jointAnglesSubSet, jointVelocitiesSubSet, jointJointTorquesSubSet, t);
      }
   }
}
