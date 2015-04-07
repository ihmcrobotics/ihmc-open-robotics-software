package us.ihmc.atlas;

import java.io.IOException;
import java.net.URI;

import us.ihmc.atlas.ros.RosAtlasAuxiliaryRobotDataPublisher;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.gfe.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import us.ihmc.utilities.ros.RosMainNode;

public class AtlasROSAPINetworkProcessor
{
   private static String defaultRosNameSpace = "/ihmc_ros/atlas";
   private static String defaultRobotModel = "ATLAS_UNPLUGGED_V5_NO_HANDS";

   private static String nodeName = "/robot_data";

   private static final boolean ENABLE_UI_PACKET_TO_ROS_CONVERTER = true;
   
   public AtlasROSAPINetworkProcessor(DRCRobotModel robotModel, String nameSpace) throws IOException
   {
      PacketCommunicator gfeCommunicator = null;
      URI rosUri = NetworkParameters.getROSURI();

      if (ENABLE_UI_PACKET_TO_ROS_CONVERTER)
      {
         gfeCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.GFE_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());
         
         DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
         networkProcessorParameters.setUseLocalControllerCommunicator(false);
         networkProcessorParameters.setUseUiModule(true);
         networkProcessorParameters.setUseGFECommunicator(true);
         DRCNetworkProcessor networkProcessor = new DRCNetworkProcessor(robotModel, networkProcessorParameters);
         new UiPacketToRosMsgRedirector(robotModel, rosUri, gfeCommunicator, networkProcessor.getPacketRouter());
      }
      else 
      {
         String kryoIP = NetworkParameters.getHost(NetworkParameterKeys.robotController);
         gfeCommunicator = PacketCommunicator.createTCPPacketCommunicatorClient(kryoIP, NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
      }

      RosMainNode rosMainNode = new RosMainNode(rosUri, nameSpace + nodeName);
      RosAtlasAuxiliaryRobotDataPublisher auxiliaryRobotDataPublisher = new RosAtlasAuxiliaryRobotDataPublisher(rosMainNode, nameSpace);
      rosMainNode.execute();

      gfeCommunicator.attachListener(RobotConfigurationData.class, auxiliaryRobotDataPublisher);
      new ThePeoplesGloriousNetworkProcessor(rosUri, gfeCommunicator, robotModel, nameSpace);
   }
   
   public static void main(String[] args) throws JSAPException, IOException
   {
      JSAP jsap = new JSAP();
      
      FlaggedOption rosNameSpace = new FlaggedOption("namespace").setLongFlag("namespace").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      rosNameSpace.setDefault(defaultRosNameSpace);

      FlaggedOption model = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(false).setStringParser(JSAP.STRING_PARSER);
      model.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      model.setDefault(defaultRobotModel);
      
      jsap.registerParameter(model);
      jsap.registerParameter(rosNameSpace);
      JSAPResult config = jsap.parse(args);

      DRCRobotModel robotModel;

      try
      {
         // This should *ALWAYS* be set to REAL_ROBOT, as this network processor is designed to run only on the real robot.
         // For sim, see AtlasROSAPISimulator
         robotModel = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), AtlasRobotModel.AtlasTarget.REAL_ROBOT, false);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect robot model " + config.getString("robotModel"));
         System.out.println(jsap.getHelp());
         return;
      }

      new AtlasROSAPINetworkProcessor(robotModel, config.getString("namespace"));
   }
}
