package us.ihmc.atlas;

import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DummyController;
import us.ihmc.iRobot.control.IRobotControlThreadManager;
import us.ihmc.utilities.net.LocalObjectCommunicator;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

public class AtlasNetworkProcessor
{
   private static String scsMachineIPAddress = DRCLocalConfigParameters.ROBOT_CONTROLLER_IP_ADDRESS;

   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      JSAP jsap = new JSAP();
      FlaggedOption scsIPFlag = new FlaggedOption("scs-ip").setLongFlag("scs-ip").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      FlaggedOption rosURIFlag = new FlaggedOption("ros-uri").setLongFlag("ros-uri").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      Switch simulateController = new Switch("simulate-controller").setShortFlag('d').setLongFlag(JSAP.NO_LONGFLAG);

      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      
      Switch runningOnRealRobot = new Switch("runningOnRealRobot").setLongFlag("realRobot").setShortFlag('r');

      robotModel.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      jsap.registerParameter(robotModel);

      jsap.registerParameter(scsIPFlag);
      jsap.registerParameter(rosURIFlag);
      jsap.registerParameter(simulateController);
      jsap.registerParameter(runningOnRealRobot);

      JSAPResult config = jsap.parse(args);

      if (config.success())
      {
    	  DRCRobotModel model;
    	  try
    	  {
    		  model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), config.getBoolean(runningOnRealRobot.getID()), true);
    	  }
    	  catch (IllegalArgumentException e)
    	  {
    		  System.err.println("Incorrect robot model " + config.getString("robotModel"));
    		  System.out.println(jsap.getHelp());
    		  
    		  return;
    	  }
    	  
         if (config.getString(scsIPFlag.getID()) != null)
         {
            scsMachineIPAddress = config.getString(scsIPFlag.getID());
         }
         else
         {
        	 scsMachineIPAddress = model.getNetworkParameters().getRobotControlComputerIP();
         }

         String rosMasterURI;
         if (config.getString(rosURIFlag.getID()) != null)
         {
            rosMasterURI = config.getString(rosURIFlag.getID());
         }
         else
         {
        	 rosMasterURI = model.getNetworkParameters().getROSHostIP();
         }

         if (config.getBoolean(simulateController.getID()) && config.getBoolean(runningOnRealRobot.getID()))
         {
            System.err
                  .println("WARNING WARNING WARNING :: Simulating DRC Controller - WILL NOT WORK ON REAL ROBOT. Do not use -d argument when running on real robot.");
            LocalObjectCommunicator objectCommunicator = new LocalObjectCommunicator();

            new DummyController(rosMasterURI, objectCommunicator, model, new IRobotControlThreadManager(objectCommunicator));
            new DRCNetworkProcessor(objectCommunicator, model);
         }
         else
         {
            new DRCNetworkProcessor(new URI(rosMasterURI), model);
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
