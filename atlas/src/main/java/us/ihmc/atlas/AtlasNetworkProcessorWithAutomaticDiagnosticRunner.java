package us.ihmc.atlas;

import java.net.URISyntaxException;

import com.martiansoftware.jsap.*;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class AtlasNetworkProcessorWithAutomaticDiagnosticRunner
{
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
        
        try
        {
           RobotTarget target;
           if(config.getBoolean(runningOnRealRobot.getID()))
           {
              target = RobotTarget.REAL_ROBOT;
           }
           else if(config.getBoolean(runningOnGazebo.getID()))
           {
             target = RobotTarget.GAZEBO;
           }
           else
           {
              target = RobotTarget.SCS;
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
        
        HumanoidNetworkProcessor networkProcessor = new HumanoidNetworkProcessor(model, PubSubImplementation.FAST_RTPS);
        LogTools.info("ROS_MASTER_URI = " + networkProcessor.getOrCreateRosURI());
        networkProcessor.setupRosModule();
        networkProcessor.setupBehaviorModule(true, true, 15.0);
        networkProcessor.setupShutdownHook();
        networkProcessor.start();
      }
      else
      {
         System.err.println("Invalid parameters");
         System.out.println(jsap.getHelp());
         return;
      }
   }
}
