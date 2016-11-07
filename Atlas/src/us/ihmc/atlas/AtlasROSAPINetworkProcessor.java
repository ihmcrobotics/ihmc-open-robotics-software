package us.ihmc.atlas;

import java.io.IOException;
import java.net.URI;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.atlas.ros.RosAtlasAuxiliaryRobotDataPublisher;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.DRCNetworkProcessor;
import us.ihmc.avatar.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;
import us.ihmc.avatar.rosAPI.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.utilities.ros.RosMainNode;

public class AtlasROSAPINetworkProcessor
{
   private static final String DEFAULT_TF_PREFIX = null;
   private static String defaultRosNameSpace = "/ihmc_ros/atlas";
   private static String defaultRobotModel = "ATLAS_UNPLUGGED_V5_NO_HANDS";

   private static String nodeName = "/robot_data";

   private static final boolean ENABLE_UI_PACKET_TO_ROS_CONVERTER = true;
   
   public AtlasROSAPINetworkProcessor(DRCRobotModel robotModel, String nameSpace, String tfPrefix) throws IOException
   {
      PacketCommunicator rosAPICommunicator = null;
      URI rosUri = NetworkParameters.getROSURI();

      if (ENABLE_UI_PACKET_TO_ROS_CONVERTER)
      {
         rosAPICommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());
         
         DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
         networkProcessorParameters.enableLocalControllerCommunicator(false);
         networkProcessorParameters.enableUiModule(true);
         networkProcessorParameters.enableROSAPICommunicator(true);
         networkProcessorParameters.enableControllerCommunicator(true);
         DRCNetworkProcessor networkProcessor = new DRCNetworkProcessor(robotModel, networkProcessorParameters);
         new UiPacketToRosMsgRedirector(robotModel, rosUri, rosAPICommunicator, networkProcessor.getPacketRouter(), defaultRosNameSpace);
      }
      else 
      {
         String kryoIP = NetworkParameters.getHost(NetworkParameterKeys.robotController);
         rosAPICommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(kryoIP, NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
      }

      RosMainNode rosMainNode = new RosMainNode(rosUri, nameSpace + nodeName);
      RosAtlasAuxiliaryRobotDataPublisher auxiliaryRobotDataPublisher = new RosAtlasAuxiliaryRobotDataPublisher(rosMainNode, nameSpace);
      rosMainNode.execute();

      rosAPICommunicator.attachListener(RobotConfigurationData.class, auxiliaryRobotDataPublisher);
      new ThePeoplesGloriousNetworkProcessor(rosUri, rosAPICommunicator, robotModel, nameSpace, tfPrefix);
   }
   
   public static void main(String[] args) throws JSAPException, IOException
   {
      JSAP jsap = new JSAP();
      
      FlaggedOption rosNameSpace = new FlaggedOption("namespace").setLongFlag("namespace").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      rosNameSpace.setDefault(defaultRosNameSpace);
      
      FlaggedOption tfPrefix = new FlaggedOption("tfPrefix").setLongFlag("tfPrefix").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      tfPrefix.setDefault(DEFAULT_TF_PREFIX);

      FlaggedOption model = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(false).setStringParser(JSAP.STRING_PARSER);
      model.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      model.setDefault(defaultRobotModel);
      
      jsap.registerParameter(tfPrefix);
      jsap.registerParameter(model);
      jsap.registerParameter(rosNameSpace);
      JSAPResult config = jsap.parse(args);

      DRCRobotModel robotModel;

      try
      {
         // This should *ALWAYS* be set to REAL_ROBOT, as this network processor is designed to run only on the real robot.
         // For sim, see AtlasROSAPISimulator
         robotModel = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), DRCRobotModel.RobotTarget.REAL_ROBOT, false);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect robot model " + config.getString("robotModel"));
         System.out.println(jsap.getHelp());
         return;
      }
      String tfPrefixArg = config.getString("tfPrefix");
      String nodeNameSpacePrefix = config.getString("namespace");
      new AtlasROSAPINetworkProcessor(robotModel, nodeNameSpacePrefix, tfPrefixArg);
   }
}
