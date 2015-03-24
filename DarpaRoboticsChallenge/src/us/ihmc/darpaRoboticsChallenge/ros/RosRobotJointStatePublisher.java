package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.ros.message.Time;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModelUtils;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.msgToPacket.IHMCRosApiMessageMap;
import us.ihmc.utilities.ros.publisher.RosJointStatePublisher;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class RosRobotJointStatePublisher implements PacketConsumer<RobotConfigurationData>, Runnable
{
   public static final String WORLD_FRAME = "world";

   private final RosTfPublisher tfPublisher;

   private final RosJointStatePublisher jointStatePublisher;
   private final ArrayList<String> nameList = new ArrayList<String>();
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final ConcurrentLinkedQueue<RobotConfigurationData> availableRobotConfigurationData = new ConcurrentLinkedQueue<RobotConfigurationData>();

   private final RigidBodyTransform transformFromHeadToMultisenseHeadRoot = new RigidBodyTransform();

   private final int jointNameHash;

   public RosRobotJointStatePublisher(SDFFullRobotModelFactory sdfFullRobotModelFactory, PacketCommunicator fieldComputer, final RosMainNode rosMainNode,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider, String rosNameSpace, RosTfPublisher tfPublisher)
   {
      SDFFullRobotModel fullRobotModel = sdfFullRobotModelFactory.createFullRobotModel();
      this.rosMainNode = rosMainNode;
      this.jointStatePublisher = new RosJointStatePublisher(false);
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.tfPublisher = tfPublisher;

      OneDoFJoint[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      for (int i = 0; i < joints.length; i++)
      {
         nameList.add(joints[i].getName());
      }

      jointNameHash = RobotConfigurationData.calculateJointNameHash(joints, fullRobotModel.getForceSensorDefinitions());

      rosMainNode.attachPublisher(rosNameSpace + IHMCRosApiMessageMap.PACKET_TO_TOPIC_MAP.get(RobotConfigurationData.class), jointStatePublisher);
      fieldComputer.attachListener(RobotConfigurationData.class, this);
      
      Thread t = new Thread(this, "RosRobotJointStatePublisher");
      t.start();
   }

   @Override
   public void receivedPacket(RobotConfigurationData robotConfigurationData)
   {
      availableRobotConfigurationData.add(robotConfigurationData);
      if(availableRobotConfigurationData.size() > 30)
      {
         availableRobotConfigurationData.clear();
      }
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
               long timeStamp = ppsTimestampOffsetProvider.adjustRobotTimeStampToRosClock(robotConfigurationData.getTimestamp());
               Time t = Time.fromNano(timeStamp);

               if (robotConfigurationData.jointNameHash != jointNameHash)
               {
                  throw new RuntimeException("Joint names do not match for RobotConfigurationData");
               }

               jointStatePublisher.publish(nameList, robotConfigurationData.getJointAngles(), robotConfigurationData.getJointVelocities(),
                     robotConfigurationData.getJointTorques(), t);

               RigidBodyTransform pelvisTransform = new RigidBodyTransform(robotConfigurationData.getOrientation(), robotConfigurationData.getTranslation());
               tfPublisher.publish(pelvisTransform, timeStamp, WORLD_FRAME, "pelvis");
               tfPublisher.publish(transformFromHeadToMultisenseHeadRoot, timeStamp, "head", "multisense/head_root");

            }
         }
         ThreadTools.sleep(1);
      }
   }
}

























