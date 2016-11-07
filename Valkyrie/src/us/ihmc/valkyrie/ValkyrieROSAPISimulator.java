package us.ihmc.valkyrie;

import java.io.IOException;
import java.util.List;
import java.util.Map;

import com.martiansoftware.jsap.JSAPException;

import org.ros.internal.message.Message;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros.ROSAPISimulator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

public class ValkyrieROSAPISimulator extends ROSAPISimulator
{
   private static final String ROBOT_NAME = "valkyrie";
   
   public ValkyrieROSAPISimulator(DRCRobotModel robotModel, DRCStartingLocation startingLocation, String nameSpace, String tfPrefix,
         boolean runAutomaticDiagnosticRoutine, boolean disableViz) throws IOException
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
      return null;
   }

   @Override protected List<Map.Entry<String, RosTopicPublisher<? extends Message>>> createCustomPublishers(String nameSpace, PacketCommunicator communicator)
   {
      return null;
   }

   public static void main(String[] args) throws JSAPException, IOException
   {
      Options opt = parseArguments(args);
      
      DRCRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
      
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
      new ValkyrieROSAPISimulator(robotModel, startingLocation, nameSpace, opt.tfPrefix, opt.runAutomaticDiagnosticRoutine, opt.disableViz);
   }
}
