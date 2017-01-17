package us.ihmc.steppr.simulation;

import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.steppr.parameters.BonoRobotModel;
import us.ihmc.tools.thread.ThreadTools;

public class StepprFlatGroundCircularWalkingTrack
{
   public static void main(String[] args)
   {
      BonoRobotModel robotModel = new BonoRobotModel(false, false);
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);


      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      double initialYaw = 0.3;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);
      boolean useVelocityAndHeadingScript = false;
      boolean cheatWithGroundHeightAtForFootstep = false;

      DRCFlatGroundWalkingTrack flatGroundWalkingTrack=new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup,
                                    useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, robotModel);
      flatGroundWalkingTrack.getAvatarSimulation().start();
      flatGroundWalkingTrack.getAvatarSimulation().simulate();
      SimulationConstructionSet scs = flatGroundWalkingTrack.getSimulationConstructionSet();
      
      scs.getVariable("RateBasedDesiredHeadingControlModule", "desiredHeadingDot").setValueFromDouble(0.1); 
      scs.getVariable("MomentumBasedControllerFactory","swingTime").setValueFromDouble(0.75); 
      scs.getVariable("MomentumBasedControllerFactory","transferTime").setValueFromDouble(0.25); 
      YoVariable<?> desiredVelocityX =  scs.getVariable("ManualDesiredVelocityControlModule", "desiredVelocityX");
      

      for(double v=0.0; v<0.6; v+=0.1)
      {
              desiredVelocityX.setValueFromDouble(v); 
              ThreadTools.sleep(10000);
      }
      
      scs.stop();
      
   }
}