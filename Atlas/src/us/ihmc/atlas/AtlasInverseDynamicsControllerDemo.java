package us.ihmc.atlas;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCInverseDynamicsControllerDemo;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

import com.martiansoftware.jsap.JSAPException;

public class AtlasInverseDynamicsControllerDemo
{
   private static final double ROBOT_FLOATING_HEIGHT = 0.3;
   private static final DRCRobotModel defaultModelForGraphicSelector = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   public static void main(String[] args) throws JSAPException
   {
      DRCRobotModel model = null;
      
      model = AtlasRobotModelFactory.selectSimulationModelFromFlag(args);
      
      if (model == null)
         model = AtlasRobotModelFactory.selectModelFromGraphicSelector(defaultModelForGraphicSelector);
      
      if (model == null)
         throw new RuntimeException("No robot model selected");
      
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);
      
      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, model.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);
      
      double initialYaw = 0.0;
      DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup = model.getDefaultRobotInitialSetup(groundHeight + ROBOT_FLOATING_HEIGHT, initialYaw);

      new DRCInverseDynamicsControllerDemo(robotInitialSetup, guiInitialSetup, scsInitialSetup, model);
   }
}
