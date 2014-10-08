package us.ihmc.darpaRoboticsChallenge.gfe;

import java.net.URI;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.utilities.net.AtomicSettableTimestampProvider;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.ros.RosMainNode;

public class ThePeoplesGloriousGFENetworkProcessor
{
   private static final String nodeName = "/Controller";
   
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final RosMainNode rosMainNode;
   
   public ThePeoplesGloriousGFENetworkProcessor(URI rosUri, ObjectCommunicator controllerCommunicationBridge, ObjectCommunicator scsSensorCommunicationBridge, DRCRobotModel robotModel, String namespace)
   {
      this.rosMainNode = new RosMainNode(rosUri, namespace + nodeName);
      ppsTimestampOffsetProvider = robotModel.getPPSTimestampOffsetProvider();
      ppsTimestampOffsetProvider.attachToRosMainNode(rosMainNode);
      
      new GFERosOutputs(rosMainNode, controllerCommunicationBridge, scsSensorCommunicationBridge, robotModel, ppsTimestampOffsetProvider, namespace, timestampProvider);
      new GFERosInputs(rosMainNode, controllerCommunicationBridge, namespace);
      rosMainNode.execute();
   }
}
