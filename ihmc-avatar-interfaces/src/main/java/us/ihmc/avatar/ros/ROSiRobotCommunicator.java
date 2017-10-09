package us.ihmc.avatar.ros;

import java.net.URI;
import java.net.URISyntaxException;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import handle_msgs.HandleControl;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ROSiRobotCommunicator extends AbstractNodeMain
{
   private static final String MASTER_URI = "http://localhost:11311";

   private URI master;

   private final SideDependentList<Publisher<handle_msgs.HandleControl>> handCommandPublishers = new SideDependentList<Publisher<HandleControl>>();
   private final SideDependentList<handle_msgs.HandleControl> handControlMessages = new SideDependentList<HandleControl>();

   private Publisher<handle_msgs.HandleControl> leftHandPublisher, rightHandPublisher;
   private handle_msgs.HandleControl leftHandControlMessage, rightHandControlMessage;

   private ConnectedNode connectedNode;

   public ROSiRobotCommunicator()
   {
      try
      {
         master = new URI(MASTER_URI);
      }
      catch (URISyntaxException e)
      {
         e.printStackTrace();
      }
   }

   public ROSiRobotCommunicator(String uri)
   {
      try
      {
         master = new URI(uri);
      }
      catch (URISyntaxException e)
      {
         e.printStackTrace();
      }
   }

   @Override
   public GraphName getDefaultNodeName()
   {
      return GraphName.of("darpaRoboticsChallenge/iRobotROSCommunicator");
   }

   @Override
   public void onStart(ConnectedNode connectedNode)
   {
      this.connectedNode = connectedNode;

      setupPublishers();

      setupMessages();
   }

   private void setupPublishers()
   {
      leftHandPublisher = connectedNode.newPublisher("/left_hand/control", handle_msgs.HandleControl._TYPE);
      rightHandPublisher = connectedNode.newPublisher("/right_hand/control", handle_msgs.HandleControl._TYPE);

      handCommandPublishers.put(RobotSide.LEFT, leftHandPublisher);
      handCommandPublishers.put(RobotSide.RIGHT, rightHandPublisher);
   }

   private void setupMessages()
   {
      leftHandControlMessage = leftHandPublisher.newMessage();
      rightHandControlMessage = rightHandPublisher.newMessage();

      handControlMessages.put(RobotSide.LEFT, leftHandControlMessage);
      handControlMessages.put(RobotSide.RIGHT, rightHandControlMessage);
   }

   public void sendHandCommand(HandDesiredConfigurationMessage packet)
   {
      HandDesiredConfigurationMessageToHandleControlMessageConverter.convertHandDesiredConfigurationMessage(packet, handControlMessages.get(packet.getRobotSide()));
      handCommandPublishers.get(packet.getRobotSide()).publish(handControlMessages.get(packet.getRobotSide()));
   }
}
