package us.ihmc.valkyrie.simulation;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCInverseDynamicsControllerDemo;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieInverseDynamicsControllerDemo
{
   public static void main(String[] args)
   {
      DRCRobotModel model = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
      double groundHeight = 0.0;
      double initialYaw = 0.0;
      DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup = model.getDefaultRobotInitialSetup(groundHeight + 0.3, initialYaw);
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, true);
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, model.getSimulateDT());
      
      new DRCInverseDynamicsControllerDemo(robotInitialSetup, guiInitialSetup, scsInitialSetup, model);
   }
}
