package us.ihmc.atlas;

import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DummyController;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

public class AtlasNetworkProcessor
{
   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      JSAP jsap = new JSAP();
      FlaggedOption scsIPFlag = new FlaggedOption("scs-ip").setLongFlag("scs-ip").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      FlaggedOption rosURIFlag = new FlaggedOption("ros-uri").setLongFlag("ros-uri").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      Switch simulateController = new Switch("simulate-controller").setShortFlag('d').setLongFlag(JSAP.NO_LONGFLAG);

      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      
      Switch runningOnRealRobot = new Switch("runningOnRealRobot").setLongFlag("realRobot");
      Switch runningOnGazebo = new Switch("runningOnGazebo").setLongFlag("gazebo");
      
      FlaggedOption leftHandHost = new FlaggedOption("leftHandHost").setLongFlag("lefthand").setShortFlag('l').setRequired(false).setStringParser(JSAP.STRING_PARSER);
      FlaggedOption rightHandHost = new FlaggedOption("rightHandHost").setLongFlag("righthand").setShortFlag('r').setRequired(false).setStringParser(JSAP.STRING_PARSER);

      robotModel.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      jsap.registerParameter(robotModel);

      jsap.registerParameter(scsIPFlag);
      jsap.registerParameter(rosURIFlag);
      jsap.registerParameter(simulateController);
      jsap.registerParameter(runningOnRealRobot);
      jsap.registerParameter(runningOnGazebo);
      jsap.registerParameter(leftHandHost);
      jsap.registerParameter(rightHandHost);

      JSAPResult config = jsap.parse(args);

      if (config.success())
      {
    	  DRCRobotModel model;
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
    	  }
    	  catch (IllegalArgumentException e)
    	  {
    		  System.err.println("Incorrect robot model " + config.getString("robotModel"));
    		  System.out.println(jsap.getHelp());
    		  
    		  return;
    	  }
    	  
    	  System.out.println("Using the " + model + " model");
    	  
    	  URI rosMasterURI;
    	  if (config.getString(rosURIFlag.getID()) != null)
    	  {
    	     rosMasterURI = new URI(config.getString(rosURIFlag.getID()));
    	  }
    	  else
    	  {
    	     rosMasterURI = model.getNetworkParameters().getRosURI();
    	  }

    	  if (config.getBoolean(simulateController.getID()) && config.getBoolean(runningOnRealRobot.getID()))
    	  {
    	     System.err
    	     .println("WARNING WARNING WARNING :: Simulating DRC Controller - WILL NOT WORK ON REAL ROBOT. Do not use -d argument when running on real robot.");
    	     KryoLocalPacketCommunicator objectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), PacketDestination.CONTROLLER.ordinal(), "AtlasNetworkProcessorSimulatedControllerCommunicator");
    	     
    	     new DummyController(rosMasterURI, objectCommunicator, model);
    	     new DRCNetworkProcessor(objectCommunicator, model);
    	  }
    	  else
    	  {
    	     new DRCNetworkProcessor(rosMasterURI, model);
    	  }
      }
      else
      {
         System.err.println("Invalid parameters");
         System.out.println(jsap.getHelp());
         return;
      }
   }
}
