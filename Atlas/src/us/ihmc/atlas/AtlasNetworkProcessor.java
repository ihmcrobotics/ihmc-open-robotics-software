package us.ihmc.atlas;

import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.atlas.ros.RosAtlasAuxiliaryRobotDataPublisher;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

public class AtlasNetworkProcessor
{
   private static final boolean ENABLE_BEHAVIOR_MODULE = false;
   private static String defaultRosNameSpace = "/ihmc_ros/atlas";
   
   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      JSAP jsap = new JSAP();

      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      
      Switch runningOnRealRobot = new Switch("runningOnRealRobot").setLongFlag("realRobot");
      Switch runningOnGazebo = new Switch("runningOnGazebo").setLongFlag("gazebo");
      
      FlaggedOption leftHandHost = new FlaggedOption("leftHandHost").setLongFlag("lefthand").setShortFlag('l').setRequired(false).setStringParser(JSAP.STRING_PARSER);
      FlaggedOption rightHandHost = new FlaggedOption("rightHandHost").setLongFlag("righthand").setShortFlag('r').setRequired(false).setStringParser(JSAP.STRING_PARSER);

      robotModel.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      jsap.registerParameter(robotModel);

      jsap.registerParameter(runningOnRealRobot);
      jsap.registerParameter(runningOnGazebo);
      jsap.registerParameter(leftHandHost);
      jsap.registerParameter(rightHandHost);

      JSAPResult config = jsap.parse(args);

      if (config.success())
      {
    	  DRCRobotModel model;
    	  
    	  DRCNetworkModuleParameters networkModuleParams = new DRCNetworkModuleParameters();
       
        networkModuleParams.enableUiModule(true);
        networkModuleParams.enableBehaviorModule(ENABLE_BEHAVIOR_MODULE);
        networkModuleParams.enableSensorModule(true);
        networkModuleParams.enableBehaviorVisualizer(true);
        networkModuleParams.setDrillDetectionModuleEnabled(true);

        URI rosuri = NetworkParameters.getROSURI();
        if(rosuri != null)
        {
           networkModuleParams.enableRosModule(true);
           networkModuleParams.setRosUri(rosuri);
           System.out.println("ROS_MASTER_URI="+rosuri);
           
            createAuxiliaryRobotDataRosPublisher(networkModuleParams, rosuri);
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
              networkModuleParams.enableHandModule(true);       
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
        
    	  
    	  networkModuleParams.enableLocalControllerCommunicator(false);
    	  
    	  new DRCNetworkProcessor(model, networkModuleParams);
      }
      else
      {
         System.err.println("Invalid parameters");
         System.out.println(jsap.getHelp());
         return;
      }
   }

   private static void createAuxiliaryRobotDataRosPublisher(DRCNetworkModuleParameters networkModuleParams, URI rosuri)
   {
      RosAtlasAuxiliaryRobotDataPublisher auxiliaryRobotDataPublisher = new RosAtlasAuxiliaryRobotDataPublisher(rosuri, defaultRosNameSpace);
      PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_AUXILIARY_ROBOT_DATA_PUBLISHER,
            new IHMCCommunicationKryoNetClassList());
      packetCommunicator.attachListener(RobotConfigurationData.class, auxiliaryRobotDataPublisher);

      networkModuleParams.addRobotSpecificModuleCommunicatorPort(NetworkPorts.ROS_AUXILIARY_ROBOT_DATA_PUBLISHER, PacketDestination.AUXILIARY_ROBOT_DATA_PUBLISHER);
   }
}
