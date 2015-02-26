package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.ArrayList;

import org.ros.message.Time;
import org.ros.time.WallTimeProvider;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModelUtils;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosJointStatePublisher;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class RosRobotJointStatePublisher implements PacketConsumer<RobotConfigurationData>
{
   private final RosJointStatePublisher jointStatePublisher;
   private final WallTimeProvider wallTime;
   private final ArrayList<String> nameList = new ArrayList<String>();
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   
   private final int jointNameHash;

   public RosRobotJointStatePublisher(FullRobotModel fullRobotModel, PacketCommunicator fieldComputer, final RosMainNode rosMainNode, PPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         String rosNameSpace)
   {
      this.rosMainNode = rosMainNode;
      this.wallTime = new WallTimeProvider();
      this.jointStatePublisher = new RosJointStatePublisher(false);
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;

      OneDoFJoint[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      for(int i = 0; i < joints.length; i++)
      {
         nameList.add(joints[i].getName());
      }
      
      jointNameHash = RobotConfigurationData.calculateJointNameHash(joints, fullRobotModel.getForceSensorDefinitions());
      
      rosMainNode.attachPublisher("/" + rosNameSpace + "/joint_states", jointStatePublisher);
      fieldComputer.attachListener(RobotConfigurationData.class, this);
   }

   @Override
   public void receivedPacket(RobotConfigurationData object)
   {
      if (rosMainNode.isStarted())
      {
         long timeStamp = ppsTimestampOffsetProvider.adjustRobotTimeStampToRosClock(object.getTimestamp());
         Time t = Time.fromNano(timeStamp);//wallTime.getCurrentTime());

         if(object.jointNameHash != jointNameHash)
         {
            throw new RuntimeException("Joint names do not match for RobotConfigurationData");
         }
         
         jointStatePublisher.publish(nameList, object.getJointAngles(), null, null, t);
      }
   }
}
