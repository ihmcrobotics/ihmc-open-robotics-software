package us.ihmc.acsell.simulation;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTrack;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.joystick.JoystickUpdater;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.steppr.hardware.visualization.StepprStandPrepSliderboard;

public class BonoFlatGroundWalkingTrack
{
   public static void main(String[] args)
   {
      boolean USE_JOYSTICK_CONTROLLER = JoystickUpdater.isJoyStickConnected();

      BonoRobotModel robotModel = new BonoRobotModel(false, false);
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);


      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      double initialYaw = 0.3;
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);
      boolean useVelocityAndHeadingScript = !USE_JOYSTICK_CONTROLLER;
      boolean cheatWithGroundHeightAtForFootstep = false;

      DRCFlatGroundWalkingTrack flatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup,
                                                            useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, robotModel,
                                                            WalkingProvider.VELOCITY_HEADING_COMPONENT);


      SimulationConstructionSet scs = flatGroundWalkingTrack.getSimulationConstructionSet();
      if (USE_JOYSTICK_CONTROLLER)
      {
         StepprStandPrepSliderboard.setupJoyStick(scs);
         flatGroundWalkingTrack.getDrcSimulation().start();
         flatGroundWalkingTrack.getDrcSimulation().simulate();
      }
   }


}
