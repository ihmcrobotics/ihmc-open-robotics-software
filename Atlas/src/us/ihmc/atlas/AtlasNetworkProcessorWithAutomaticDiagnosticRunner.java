package us.ihmc.atlas;

import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketClientEndPointCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DummyController;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

public class AtlasNetworkProcessorWithAutomaticDiagnosticRunner
{
   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      JSAP jsap = new JSAP();
      Switch simulateController = new Switch("simulate-controller").setShortFlag('d').setLongFlag(JSAP.NO_LONGFLAG);

      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      
      Switch runningOnRealRobot = new Switch("runningOnRealRobot").setLongFlag("realRobot");
      Switch runningOnGazebo = new Switch("runningOnGazebo").setLongFlag("gazebo");
      
      FlaggedOption leftHandHost = new FlaggedOption("leftHandHost").setLongFlag("lefthand").setShortFlag('l').setRequired(false).setStringParser(JSAP.STRING_PARSER);
      FlaggedOption rightHandHost = new FlaggedOption("rightHandHost").setLongFlag("righthand").setShortFlag('r').setRequired(false).setStringParser(JSAP.STRING_PARSER);

      robotModel.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      jsap.registerParameter(robotModel);

      jsap.registerParameter(simulateController);
      jsap.registerParameter(runningOnRealRobot);
      jsap.registerParameter(runningOnGazebo);
      jsap.registerParameter(leftHandHost);
      jsap.registerParameter(rightHandHost);

      JSAPResult config = jsap.parse(args);

      if (config.success())
      {
        DRCRobotModel model;
        
        int communicatorId = PacketDestination.CONTROLLER.ordinal();
        IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
        DRCNetworkModuleParameters networkModuleParams = new DRCNetworkModuleParameters();
       
        networkModuleParams.setUseBehaviorModule(true);
        networkModuleParams.setRunAutomaticDiagnostic(true);

        URI rosuri = NetworkParameters.getROSURI();
        if(rosuri != null)
        {
           networkModuleParams.setUseRosModule(true);
           networkModuleParams.setRosUri(rosuri);
           System.out.println("ROS_MASTER_URI="+rosuri);
        }
        try
        {
           AtlasTarget target;
           if(config.getBoolean(runningOnRealRobot.getID()))
           {
              target = AtlasTarget.REAL_ROBOT;
           }
           else if(config.getBoolean(runningOnGazebo.getID()))
           {
             target = AtlasTarget.GAZEBO;
           }
           else
           {
              target = AtlasTarget.SIM;
           }
           model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), target, true);
           if(model.getHandModel()!=null)
              networkModuleParams.setUseHandModule(true);       
        }
        catch (IllegalArgumentException e)
        {
           System.err.println("Incorrect robot model " + config.getString("robotModel"));
           System.out.println(jsap.getHelp());
           
           return;
        }
        
        System.out.println("Using the " + model + " model");
        
        URI rosMasterURI = NetworkParameters.getROSURI();
        networkModuleParams.setRosUri(rosMasterURI);
        
        if (config.getBoolean(simulateController.getID()) && config.getBoolean(runningOnRealRobot.getID()))
        {
           System.err
           .println("WARNING WARNING WARNING :: Simulating DRC Controller - WILL NOT WORK ON REAL ROBOT. Do not use -d argument when running on real robot.");
           KryoLocalPacketCommunicator dummyControllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), PacketDestination.CONTROLLER.ordinal(), "Atlas_Dummy_Controller_Communicator");
           
           new DummyController(rosMasterURI, dummyControllerCommunicator, model);
           networkModuleParams.setControllerCommunicator(dummyControllerCommunicator);
        }
        else
        {
           String controllerKryoServerIp = NetworkParameters.getHost(NetworkParameterKeys.robotController);
           int tcpPort = NetworkConfigParameters.NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT;
           KryoPacketCommunicator realRobotControllerConnection = new KryoPacketClientEndPointCommunicator(controllerKryoServerIp, tcpPort, netClassList, communicatorId, "Atlas_Controller_Endpoint");
           networkModuleParams.setControllerCommunicator(realRobotControllerConnection);
        }
        
        new DRCNetworkProcessor(model, networkModuleParams);
      }
      else
      {
         System.err.println("Invalid parameters");
         System.out.println(jsap.getHelp());
         return;
      }
   }
}
