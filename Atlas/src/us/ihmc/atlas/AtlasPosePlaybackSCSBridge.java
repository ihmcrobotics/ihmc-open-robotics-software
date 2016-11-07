package us.ihmc.atlas;

import java.io.IOException;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.posePlayback.PosePlaybackSCSBridge;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;

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
            model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), DRCRobotModel.RobotTarget.SCS, false);
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

      HumanoidFloatingRootJointRobot sdfRobot = model.createHumanoidFloatingRootJointRobot(false);
      FullHumanoidRobotModel fullRobotModel = model.createFullRobotModel();
      FullHumanoidRobotModel fullRobotModelForSlider = model.createFullRobotModel();

      new PosePlaybackSCSBridge(sdfRobot, fullRobotModel, fullRobotModelForSlider, model.getControllerDT());
   }
}
