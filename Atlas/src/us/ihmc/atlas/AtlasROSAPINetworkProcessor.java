package us.ihmc.atlas;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.gfe.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.utilities.net.KryoObjectClient;
import us.ihmc.utilities.net.ObjectCommunicator;

import java.io.IOException;
import java.net.URI;

public class AtlasROSAPINetworkProcessor
{
   private static String defaultRosNameSpace = "atlas";
   private static String defaultRobotModel = "DRC_NO_HANDS";

   public AtlasROSAPINetworkProcessor(DRCRobotModel robotModel, String nameSpace) throws IOException
   {
      String kryoIP = robotModel.getNetworkParameters().getRobotControlComputerIP();
      ObjectCommunicator controllerCommunicator = new KryoObjectClient(kryoIP, NetworkConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT,
            new IHMCCommunicationKryoNetClassList());
      ((KryoObjectClient) controllerCommunicator).setReconnectAutomatically(true);
      controllerCommunicator.connect();

      URI rosUri = robotModel.getNetworkParameters().getRosURI();

      new ThePeoplesGloriousNetworkProcessor(rosUri, controllerCommunicator, robotModel, nameSpace);
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
