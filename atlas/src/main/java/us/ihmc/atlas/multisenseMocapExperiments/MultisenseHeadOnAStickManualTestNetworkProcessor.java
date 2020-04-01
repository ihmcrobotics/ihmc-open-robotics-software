package us.ihmc.atlas.multisenseMocapExperiments;

import java.io.IOException;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.atlas.AtlasRobotModelFactory;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

/**
 * Network Processor to run large scale multisense manual testing
 */
public class MultisenseHeadOnAStickManualTestNetworkProcessor
{
   private static String defaultRosNameSpace = "/ihmc_ros/atlas";
   private static String defaultRobotModel = "ATLAS_UNPLUGGED_V5_NO_HANDS";

   public MultisenseHeadOnAStickManualTestNetworkProcessor(DRCRobotModel robotModel, String nameSpace) throws IOException
   {
      HumanoidNetworkProcessor networkProcessor = new HumanoidNetworkProcessor(robotModel, PubSubImplementation.FAST_RTPS);
      networkProcessor.setupRosModule();
      networkProcessor.setupSensorModule();
      networkProcessor.setupShutdownHook();
      networkProcessor.start();
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
         robotModel = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), RobotTarget.HEAD_ON_A_STICK, false);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect robot model " + config.getString("robotModel"));
         System.out.println(jsap.getHelp());
         return;
      }

      new MultisenseHeadOnAStickManualTestNetworkProcessor(robotModel, config.getString("namespace"));
   }
}
