package us.ihmc.atlas;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class AtlasObjectDetectionModulalStarter
{
   public static void main(String[] args) throws JSAPException
   {
      JSAP jsap = new JSAP();

      FlaggedOption robotModelFlag = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true)
                                                                    .setStringParser(JSAP.STRING_PARSER);

      robotModelFlag.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      jsap.registerParameter(robotModelFlag);

      JSAPResult config = jsap.parse(args);

      DRCRobotModel robotModel;

      try
      {
         robotModel = AtlasRobotModelFactory.createDRCRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString(), RobotTarget.REAL_ROBOT, false);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect robot model " + config.getString("robotModel"));
         System.out.println(jsap.getHelp());

         return;
      }

      PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
      new ObjectDetectorToolboxModule(robotModel.getSimpleRobotName(),robotModel.createFullRobotModel(),robotModel.getLogModelProvider(),pubSubImplementation);
   }
}
