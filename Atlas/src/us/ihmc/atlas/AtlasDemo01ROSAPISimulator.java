package us.ihmc.atlas;

import java.io.IOException;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.martiansoftware.jsap.JSAPException;

import ihmc_msgs.HandDesiredConfigurationRosMessage;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros.ROSAPISimulator;
import us.ihmc.avatar.ros.subscriber.IHMCMsgToPacketSubscriber;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

public class AtlasDemo01ROSAPISimulator extends ROSAPISimulator
{
   private static final String ROBOT_NAME = "atlas";
   private static final String DEFAULT_ROBOT_MODEL = "ATLAS_UNPLUGGED_V5_NO_HANDS";
   
   public AtlasDemo01ROSAPISimulator(DRCRobotModel robotModel, DRCStartingLocation startingLocation, String nameSpace, String tfPrefix,
         boolean runAutomaticDiagnosticRoutine, boolean disableViz,
         List<Map.Entry<String, RosTopicSubscriberInterface<? extends Message>>> customSubscribers,
         List<Map.Entry<String, RosTopicPublisher<? extends Message>>> customPublishers) throws IOException
   {
      super(robotModel, startingLocation, nameSpace, tfPrefix, runAutomaticDiagnosticRoutine, disableViz);
   }

   @Override
   protected CommonAvatarEnvironmentInterface createEnvironment()
   {
      return new DefaultCommonAvatarEnvironment();
   }

   @Override protected List<Map.Entry<String, RosTopicSubscriberInterface<? extends Message>>> createCustomSubscribers(String nameSpace, PacketCommunicator communicator)
   {
      List<Map.Entry<String, RosTopicSubscriberInterface<? extends Message>>> subscribers = new ArrayList<>();
      MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

      if(robotModel.getDRCHandType().isHandSimulated())
      {
         HandDesiredConfigurationRosMessage message = messageFactory.newFromType("ihmc_msgs/HandDesiredConfigurationRosMessage");
         RosTopicSubscriberInterface<HandDesiredConfigurationRosMessage> sub = IHMCMsgToPacketSubscriber
               .createIHMCMsgToPacketSubscriber(message, communicator, PacketDestination.CONTROLLER.ordinal());
         Map.Entry<String, RosTopicSubscriberInterface<? extends Message>> pair = new AbstractMap.SimpleEntry<String, RosTopicSubscriberInterface<? extends Message>>(nameSpace + "/control/finger_state", sub);
         subscribers.add(pair);
      }

      return subscribers;
   }

   @Override protected List<Map.Entry<String, RosTopicPublisher<? extends Message>>> createCustomPublishers(String nameSpace, PacketCommunicator communicator)
   {
      return null;
   }

   public static void main(String[] args) throws JSAPException, IOException
   {
      Options opt = parseArguments(args);
      
      DRCRobotModel robotModel;
      try
      {
         if (opt.robotModel.equals(DEFAULT_STRING))
         {
            robotModel = AtlasRobotModelFactory.createDRCRobotModel(DEFAULT_ROBOT_MODEL, DRCRobotModel.RobotTarget.SCS, false);
         }
         else
         {
            robotModel = AtlasRobotModelFactory.createDRCRobotModel(opt.robotModel, DRCRobotModel.RobotTarget.SCS, false);
         }
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect robot model " + opt.robotModel);
         System.out.println("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
         return;
      }
      
      DRCStartingLocation startingLocation;
      try
      {
         startingLocation = DRCObstacleCourseStartingLocation.valueOf(opt.startingLocation);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect starting location " + opt.startingLocation);
         System.out.println("Starting locations: " + DRCObstacleCourseStartingLocation.optionsToString());
         return;
      }
      
      String nameSpace = opt.nameSpace + "/" + ROBOT_NAME;
      new AtlasDemo01ROSAPISimulator(robotModel, startingLocation, nameSpace, opt.tfPrefix, opt.runAutomaticDiagnosticRoutine, opt.disableViz, null, null);
   }
}
