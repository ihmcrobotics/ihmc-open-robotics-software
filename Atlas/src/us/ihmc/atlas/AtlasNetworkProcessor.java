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
import us.ihmc.utilities.net.ObjectCommunicator;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

public class AtlasNetworkProcessor
{
   private static String scsMachineIPAddress = DRCLocalConfigParameters.ROBOT_CONTROLLER_IP_ADDRESS;
   private static String rosMasterURI = DRCConfigParameters.ROS_MASTER_URI;

   public static void main(String[] args) throws URISyntaxException, JSAPException
   {
      JSAP jsap = new JSAP();
      FlaggedOption scsIPFlag =
         new FlaggedOption("scs-ip").setLongFlag("scs-ip").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false).setStringParser(JSAP.STRING_PARSER);
      FlaggedOption rosURIFlag =
         new FlaggedOption("ros-uri").setLongFlag("ros-uri").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false).setStringParser(JSAP.STRING_PARSER);
      Switch simulateController = new Switch("simulate-controller").setShortFlag('d').setLongFlag(JSAP.NO_LONGFLAG);

      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);

      robotModel.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      jsap.registerParameter(robotModel);

      jsap.registerParameter(scsIPFlag);
      jsap.registerParameter(rosURIFlag);
      jsap.registerParameter(simulateController);

      JSAPResult config = jsap.parse(args);

      if (config.success())
      {
         if (config.getString(scsIPFlag.getID()) != null)
         {
            scsMachineIPAddress = config.getString(scsIPFlag.getID());
         }

         if (config.getString(rosURIFlag.getID()) != null)
         {
            rosMasterURI = config.getString(rosURIFlag.getID());
         }

         DRCRobotModel model;
         try
         {
            model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), DRCLocalConfigParameters.RUNNING_ON_REAL_ROBOT, true);
         }
         catch (IllegalArgumentException e)
         {
            System.err.println("Incorrect robot model " + config.getString("robotModel"));
            System.out.println(jsap.getHelp());

            return;
         }


         if (config.getBoolean(simulateController.getID()))
         {
            System.err.println(
                "WARNING WARNING WARNING :: Simulating DRC Controller - WILL NOT WORK ON REAL ROBOT. Do not use -d argument when running on real robot.");
            ObjectCommunicator objectCommunicator = new LocalObjectCommunicator();

            new DummyController(rosMasterURI, objectCommunicator, model, new IRobotControlThreadManager(objectCommunicator));
            new DRCNetworkProcessor(new URI(rosMasterURI), objectCommunicator, model);
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
