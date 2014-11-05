package us.ihmc.atlas;

import java.io.IOException;
import java.net.URI;

import com.martiansoftware.jsap.*;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseSimulation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.gfe.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;

public class AtlasROSAPISimulator
{
   private static String defaultRosNameSpace = "atlas";
   private static String defaultRobotModel = "DRC_NO_HANDS";
   
   public AtlasROSAPISimulator(DRCRobotModel robotModel, String nameSpace) throws IOException
   {
      boolean initializeEstimatorToActual = false;
      String scriptToLoad = null;
      CommonAvatarEnvironmentInterface environment = new DRCDemo01NavigationEnvironment();
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);
      
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false);
      
      ObjectCommunicator controllerCommunicator = new LocalObjectCommunicator();
      GlobalDataProducer dataProducer = new GlobalDataProducer(controllerCommunicator);

      DRCObstacleCourseSimulation simulation = new DRCObstacleCourseSimulation(environment, scriptToLoad, dataProducer, robotInitialSetup,
            guiInitialSetup, robotModel.getControllerDT(), initializeEstimatorToActual, robotModel);

      simulation.getSimulation().simulate();

      URI rosUri = robotModel.getNetworkParameters().getRosURI();

      ObjectCommunicator sensorCommunicator = simulation.getLocalObjectCommunicator();
      new ThePeoplesGloriousNetworkProcessor(rosUri, controllerCommunicator, sensorCommunicator, robotModel, nameSpace);
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
         robotModel = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), AtlasRobotModel.AtlasTarget.SIM, false);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect robot model " + config.getString("robotModel"));
         System.out.println(jsap.getHelp());
         return;
      }

      new AtlasROSAPISimulator(robotModel, config.getString("namespace"));
   }
}
