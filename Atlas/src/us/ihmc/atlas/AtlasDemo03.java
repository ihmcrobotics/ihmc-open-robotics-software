package us.ihmc.atlas;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.initialSetup.VRCTask1InVehicleHovering;
import us.ihmc.atlas.parameters.AtlasDrivingControllerParameters;
import us.ihmc.atlas.parameters.AtlasRobotMultiContactControllerParameters;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
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
   public AtlasDemo03(DRCGuiInitialSetup guiInitialSetup, AutomaticSimulationRunner automaticSimulationRunner, double timePerRecordTick,
         int simulationDataBufferSize, DRCRobotModel robotModel, DRCRobotInitialSetup<SDFRobot> robotInitialSetup,
         WalkingControllerParameters drivingControllerParameters, WalkingControllerParameters drcRobotMultiContactParameters)
   {
      super(guiInitialSetup, automaticSimulationRunner, timePerRecordTick, simulationDataBufferSize, robotModel, robotInitialSetup, drivingControllerParameters, drcRobotMultiContactParameters);
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
      
      AutomaticSimulationRunner automaticSimulationRunner = null;

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false);

      double timePerRecordTick = DRCConfigParameters.CONTROL_DT;
      int simulationDataBufferSize = 16000;
//      String ipAddress = null;
//      int portNumber = -1;
      
      WalkingControllerParameters drivingControllerParameters = new AtlasDrivingControllerParameters();
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = new VRCTask1InVehicleHovering(0.0); // new VRCTask1InVehicleInitialSetup(-0.03); // DrivingDRCRobotInitialSetup();
      WalkingControllerParameters drcRobotMultiContactParameters = new AtlasRobotMultiContactControllerParameters();
      new AtlasDemo03(guiInitialSetup, automaticSimulationRunner, timePerRecordTick, simulationDataBufferSize, model, robotInitialSetup, drivingControllerParameters, drcRobotMultiContactParameters);
   }
}
