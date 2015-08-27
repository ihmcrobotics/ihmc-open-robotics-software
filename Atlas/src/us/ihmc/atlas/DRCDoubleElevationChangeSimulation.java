package us.ihmc.atlas;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTrack;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.util.ground.DoubleStepGroundProfile;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

public class DRCDoubleElevationChangeSimulation
{   
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
            model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), DRCRobotModel.RobotTarget.SCS, false);
         }
         else
         {
            System.out.println("Enter a robot model.");
            return;
         }
      }
      catch (JSAPException e)
      {
         e.printStackTrace();
         return;
      }
      
      final double stepHeight = 0.05;
      final boolean stepUp = false;
      
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);

      final double groundHeight = 0.0;
      double elevationChange = 0.0;
      if(stepUp) elevationChange = Math.abs(stepHeight);
      else elevationChange = - Math.abs(stepHeight);
      
      GroundProfile3D groundProfile = new DoubleStepGroundProfile(-2.0, 2.0, 0.53, 0.9, elevationChange, 0.0);
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, model.getSimulateDT());    //(new FlatGroundProfile(groundHeight), robotInterface.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      
      DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup = model.getDefaultRobotInitialSetup(groundHeight,0);

      boolean useVelocityAndHeadingScript = false;
      boolean cheatWithGroundHeightAtForFootstep = false;

      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, 
                                                               useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, model);
      
      ((BooleanYoVariable) drcFlatGroundWalkingTrack.getSimulationConstructionSet().getVariable("walk")).set(true);
      ((DoubleYoVariable) drcFlatGroundWalkingTrack.getSimulationConstructionSet().getVariable("desiredVelocityX")).set(1.0);
   }

   
}
