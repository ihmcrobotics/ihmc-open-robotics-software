package us.ihmc.valkyrie.simulation;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCInverseDynamicsControllerDemo;
import us.ihmc.darpaRoboticsChallenge.DRCSimulatedRobotInterface;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.PlainDRCRobot;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile;
import us.ihmc.valkyrie.ValkyrieRobotModel;

import com.yobotics.simulationconstructionset.util.FlatGroundProfile;

public class ValkyrieInverseDynamicsControllerDemo
{
   public static void main(String[] args)
   {
      DRCRobotModel model = new ValkyrieRobotModel(false);
      DRCSimulatedRobotInterface robotInterface = new PlainDRCRobot(model);
      double groundHeight = 0.0;
      double initialYaw = 0.0;
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = model.getDefaultRobotInitialSetup(groundHeight + 0.3, initialYaw);
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, true);
      GroundProfile groundProfile = new FlatGroundProfile(groundHeight);
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotInterface.getSimulateDT());
      AutomaticSimulationRunner automaticSimulationRunner = null;
      
      new DRCInverseDynamicsControllerDemo(robotInterface, robotInitialSetup, guiInitialSetup, scsInitialSetup, automaticSimulationRunner, DRCConfigParameters.CONTROL_DT, 16000, model);
   }
}
