package us.ihmc.atlas;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import com.martiansoftware.jsap.JSAPException;

import ihmc_msgs.FingerStatePacketMessage;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.message.MessageFactoryProvider;
import org.ros.node.NodeConfiguration;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.darpaRoboticsChallenge.DRCSCStartingLocations;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.ROSAPISimulator;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCFinalsEnvironment;
import us.ihmc.darpaRoboticsChallenge.ros.subscriber.IHMCMsgToPacketSubscriber;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

public class AtlasFinalsROSAPISimulator extends ROSAPISimulator
{
   private static final String ROBOT_NAME = "atlas";
   private static final String DEFAULT_ROBOT_MODEL = "ATLAS_UNPLUGGED_V5_NO_HANDS";
   
   private static final boolean CREATE_DOOR = true;
   private static final boolean CREATE_DRILL = true;
   private static final boolean CREATE_VALVE = true;
   private static final boolean CREATE_WALKING = true;
   private static final boolean CREATE_STAIRS = true;
   
   public AtlasFinalsROSAPISimulator(DRCRobotModel robotModel, DRCStartingLocation startingLocation, String nameSpace, String tfPrefix,
         boolean runAutomaticDiagnosticRoutine, boolean disableViz) throws IOException
   {
      super(robotModel, startingLocation, nameSpace, tfPrefix, runAutomaticDiagnosticRoutine, disableViz);
   }

   @Override
   protected CommonAvatarEnvironmentInterface createEnvironment()
   {
      return new DRCFinalsEnvironment(CREATE_DOOR, CREATE_DRILL, CREATE_VALVE, CREATE_WALKING, CREATE_STAIRS);
   }

   @Override protected List<Map.Entry<String, RosTopicSubscriberInterface<? extends Message>>> createCustomSubscribers(PacketCommunicator communicator)
   {
      return null;
   }

   @Override protected List<Map.Entry<String, RosTopicPublisher<? extends Message>>> createCustomPublishers(PacketCommunicator communicator)
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
         startingLocation = DRCSCStartingLocations.valueOf(opt.startingLocation);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect starting location " + opt.startingLocation);
         System.out.println("Starting locations: " + DRCSCStartingLocations.optionsToString());
         return;
      }
      
      String nameSpace = opt.nameSpace + "/" + ROBOT_NAME;

      new AtlasFinalsROSAPISimulator(robotModel, startingLocation, nameSpace, opt.tfPrefix, opt.runAutomaticDiagnosticRoutine, opt.disableViz);
   }
}
