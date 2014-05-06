package us.ihmc.atlas;

import java.io.IOException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.posePlayback.PosePlaybackSCSBridge;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPResult;

public class AtlasPosePlaybackSCSBridge
{
   public static void main(String[] args) throws IOException
   {
      // Flag to set robot model
      JSAP jsap = new JSAP();
      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      robotModel.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());

      DRCRobotModel model;
      try
      {
         jsap.registerParameter(robotModel);

         JSAPResult config = jsap.parse(args);

         if (config.success())
         {
            model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"));
         }
         else
         {
            System.out.println("Enter a robot model.");

            return;
         }
      }
      catch (Exception e)
      {
         System.out.println("Robot model not found");
         e.printStackTrace();

         return;
      }

      DRCRobotJointMap jointMap = model.getJointMap();
      JaxbSDFLoader loader = model.getJaxbSDFLoader(false);
      SDFRobot sdfRobot = loader.createRobot(jointMap, false);
      FullRobotModel fullRobotModel = loader.createFullRobotModel(jointMap);
      SDFFullRobotModel fullRobotModelForSlider = loader.createFullRobotModel(jointMap);

      new PosePlaybackSCSBridge(sdfRobot, fullRobotModel, fullRobotModelForSlider);
   }
}
