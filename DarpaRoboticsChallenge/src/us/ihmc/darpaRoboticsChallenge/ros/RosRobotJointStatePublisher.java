package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.ArrayList;

import org.ros.message.Time;
import org.ros.time.WallTimeProvider;

import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosJointStatePublisher;

public class RosRobotJointStatePublisher implements PacketConsumer<RobotConfigurationData>
{
   private final RosJointStatePublisher jointStatePublisher;
   private final WallTimeProvider wallTime;
   private final ArrayList<String> nameList = new ArrayList<String>();
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;

   public RosRobotJointStatePublisher(PacketCommunicator fieldComputer, final RosMainNode rosMainNode, PPSTimestampOffsetProvider ppsTimestampOffsetProvider,
         String rosNameSpace)
   {
      this.rosMainNode = rosMainNode;
      this.wallTime = new WallTimeProvider();
      this.jointStatePublisher = new RosJointStatePublisher(false);
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;

      rosMainNode.attachPublisher("/" + rosNameSpace + "/joint_states", jointStatePublisher);
      fieldComputer.attachListener(RobotConfigurationData.class, this);
   }

   @Override
   public void receivedPacket(RobotConfigurationData object)
   {
      if (rosMainNode.isStarted())
      {

         long timeStamp = ppsTimestampOffsetProvider.adjustRobotTimeStampToRosClock(object.getSimTime());
         Time t = Time.fromNano(timeStamp);//wallTime.getCurrentTime());

         updateNameList(object);

         jointStatePublisher.publish(nameList, object.getJointAngles(), null, null, t);
      }
   }

   private void updateNameList(RobotConfigurationData object)
   {
      String[] jointNames = object.getJointNames();
      for (int i = 0; i < jointNames.length; i++)
      {
         if (i >= nameList.size())
            nameList.add(jointNames[i]);
         else
            nameList.set(i, jointNames[i]);
      }
      
      // Shrink the list to the size of jointNames which shouldn't change
      for (int i = jointNames.length; i > nameList.size(); i--)
      {
         nameList.remove(i);
      }
   }
}
