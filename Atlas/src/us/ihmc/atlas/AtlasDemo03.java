package us.ihmc.atlas;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.atlas.initialSetup.VRCTask1InVehicleHovering;
import us.ihmc.darpaRoboticsChallenge.DRCDemo03;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;

import com.martiansoftware.jsap.JSAPException;

public class AtlasDemo03 extends DRCDemo03
{
   public AtlasDemo03(DRCGuiInitialSetup guiInitialSetup,
         DRCRobotModel robotModel, DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup)
   {
      super(guiInitialSetup, robotModel, robotInitialSetup);
   }

   public static void main(String[] args) throws JSAPException
   {
      DRCRobotModel defaultModelForGraphicSelector = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_INVISIBLE_CONTACTABLE_PLANE_HANDS, AtlasRobotModel.AtlasTarget.SIM, false);

      DRCRobotModel model = null;
      model = AtlasRobotModelFactory.selectSimulationModelFromFlag(args);
      
      if (model == null)
         model = AtlasRobotModelFactory.selectModelFromGraphicSelector(defaultModelForGraphicSelector);

      if (model == null)
          throw new RuntimeException("No robot model selected");


      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false);

//      String ipAddress = null;
//      int portNumber = -1;
      
      DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup = new VRCTask1InVehicleHovering(0.0); // new VRCTask1InVehicleInitialSetup(-0.03); // DrivingDRCRobotInitialSetup();
      new AtlasDemo03(guiInitialSetup, model, robotInitialSetup);
   }
}
