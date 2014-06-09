package us.ihmc.atlas;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.initialSetup.VRCTask1InVehicleHovering;
import us.ihmc.darpaRoboticsChallenge.DRCDemo03;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

public class AtlasDemo03 extends DRCDemo03
{
   public AtlasDemo03(DRCGuiInitialSetup guiInitialSetup,
         DRCRobotModel robotModel, DRCRobotInitialSetup<SDFRobot> robotInitialSetup)
   {
      super(guiInitialSetup, robotModel, robotInitialSetup);
   }

   public static void main(String[] args) throws JSAPException
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
            model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), false, false);
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
     

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false);

//      String ipAddress = null;
//      int portNumber = -1;
      
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = new VRCTask1InVehicleHovering(0.0); // new VRCTask1InVehicleInitialSetup(-0.03); // DrivingDRCRobotInitialSetup();
      new AtlasDemo03(guiInitialSetup, model, robotInitialSetup);
   }
}
