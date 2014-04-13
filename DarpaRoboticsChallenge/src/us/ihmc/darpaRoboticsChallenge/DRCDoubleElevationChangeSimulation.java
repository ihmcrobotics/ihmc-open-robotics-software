package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.initialSetup.SquaredUpDRCRobotInitialSetup;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModelFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.PlainDRCRobot;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.util.DoubleStepGroundProfile;

public class DRCDoubleElevationChangeSimulation
{   
   public static void main(String[] args) throws JSAPException
   {
      // Flag to set robot model
      JSAP jsap = new JSAP();
      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      robotModel.setHelp("Robot models: " + DRCRobotModelFactory.robotModelsToString());
      
      DRCRobotModel model;
      try
      {
         jsap.registerParameter(robotModel);

         JSAPResult config = jsap.parse(args);

         if (config.success())
         {
            model = DRCRobotModelFactory.CreateDRCRobotModel(config.getString("robotModel"));
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
      
      AutomaticSimulationRunner automaticSimulationRunner = null;
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);

      DRCRobotInterface robotInterface = new PlainDRCRobot(model);
      final double groundHeight = 0.0;
      double elevationChange = 0.0;
      if(stepUp) elevationChange = Math.abs(stepHeight);
      else elevationChange = - Math.abs(stepHeight);
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(new DoubleStepGroundProfile(-2.0, 2.0, 0.53, 0.9, elevationChange, 0.0), robotInterface.getSimulateDT());    //(new FlatGroundProfile(groundHeight), robotInterface.getSimulateDT());
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = new SquaredUpDRCRobotInitialSetup(groundHeight);

      boolean useVelocityAndHeadingScript = false;
      boolean cheatWithGroundHeightAtForFootstep = false;

      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInterface, robotInitialSetup, guiInitialSetup, scsInitialSetup, 
                                                               useVelocityAndHeadingScript, automaticSimulationRunner, DRCConfigParameters.CONTROL_DT, 16000,
                                                               cheatWithGroundHeightAtForFootstep,model);
      
      ((BooleanYoVariable) drcFlatGroundWalkingTrack.getSimulationConstructionSet().getVariable("walk")).set(true);
      ((DoubleYoVariable) drcFlatGroundWalkingTrack.getSimulationConstructionSet().getVariable("desiredVelocityX")).set(1.0);
   }

   
}
