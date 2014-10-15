package us.ihmc.atlas;

import java.io.IOException;
import java.net.URI;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseSimulation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.gfe.ThePeoplesGloriousGFENetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

public class AtlasGFENetworkProcessor
{
   private static String defaultRosNameSpace = "atlas";
   
   public AtlasGFENetworkProcessor(DRCRobotModel robotModel, String nameSpace)
   {
      boolean initializeEstimatorToActual = false;
      String scriptToLoad = null;
      CommonAvatarEnvironmentInterface environment = new DRCDemo01NavigationEnvironment();
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
      
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false);
      
      ObjectCommunicator controllerCommunicator = new LocalObjectCommunicator();
      GlobalDataProducer dataProducer = new GlobalDataProducer(controllerCommunicator);
      
      DRCObstacleCourseSimulation simulation = new DRCObstacleCourseSimulation(environment.getTerrainObject3D(), scriptToLoad, dataProducer, robotInitialSetup,
            guiInitialSetup, robotModel.getControllerDT(), initializeEstimatorToActual, robotModel);

      simulation.getSimulation().simulate();
      
      URI rosUri = robotModel.getNetworkParameters().getRosURI();
      
      ObjectCommunicator sensorCommunicator = simulation.getLocalObjectCommunicator();
      new ThePeoplesGloriousGFENetworkProcessor(rosUri, controllerCommunicator, sensorCommunicator, robotModel, nameSpace);
   }
   
   public static void main(String args[]) throws JSAPException, IOException
   {
      
      JSAP jsap = new JSAP();
      
      FlaggedOption rosNameSpace = new FlaggedOption("namespace").setLongFlag("namespace").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      rosNameSpace.setDefault(defaultRosNameSpace);

      FlaggedOption model = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      model.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      
      jsap.registerParameter(model);
      jsap.registerParameter(rosNameSpace);
      JSAPResult config = jsap.parse(args);

      if (config.success())
      {
         
        DRCRobotModel robotModel;
        try
        {
           robotModel = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), false, false);
        }
        catch (IllegalArgumentException e)
        {
           System.err.println("Incorrect robot model " + config.getString("robotModel"));
           System.out.println(jsap.getHelp());
           return;
        }
        
        new AtlasGFENetworkProcessor(robotModel, config.getString("namespace"));
        
      } else {
         System.err.println("Invalid parameters");
         System.out.println(jsap.getHelp());
         return;
      }
   }
}
