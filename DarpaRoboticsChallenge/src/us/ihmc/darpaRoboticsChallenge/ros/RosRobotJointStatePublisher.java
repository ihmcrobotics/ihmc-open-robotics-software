package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.ArrayList;

import org.ros.message.Time;
import org.ros.time.WallTimeProvider;

import us.ihmc.communication.producers.DRCJointConfigurationData;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;
import us.ihmc.utilities.ros.RosJointStatePublisher;
import us.ihmc.utilities.ros.RosMainNode;

public class RosRobotJointStatePublisher implements ObjectConsumer<DRCJointConfigurationData>{
   
   private final RosJointStatePublisher jointStatePublisher;
   private final WallTimeProvider wallTime;
   private final ArrayList<String> nameList = new ArrayList<String>();
   private final RosMainNode rosMainNode;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   
   public RosRobotJointStatePublisher(ObjectCommunicator fieldComputer,final RosMainNode rosMainNode, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, String[] orderedJointNames, String rosNameSpace )
   {
      this.rosMainNode = rosMainNode;
      this.wallTime = new WallTimeProvider();
      this.jointStatePublisher = new RosJointStatePublisher(false);
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      
      for (int i = 0; i < orderedJointNames.length; i++)
      {
         nameList.add(orderedJointNames[i]);
      }
      rosMainNode.attachPublisher("/" + rosNameSpace + "/joint_states", jointStatePublisher);
      fieldComputer.attachListener(DRCJointConfigurationData.class, this);
   }
   
   @Override
   public void consumeObject(DRCJointConfigurationData object)
   {
      if(rosMainNode.isStarted()){
         
         long timeStamp = ppsTimestampOffsetProvider.adjustRobotTimeStampToRosClock(object.getSimTime());
         Time t = Time.fromNano(timeStamp);//wallTime.getCurrentTime());  
         jointStatePublisher.publish(nameList, object.getJointAngles(), null, null, t);
      }
   }
}
