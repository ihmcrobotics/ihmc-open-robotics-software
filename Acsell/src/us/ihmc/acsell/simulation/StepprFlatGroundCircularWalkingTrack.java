package us.ihmc.acsell.simulation;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTrack;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

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
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);
      boolean useVelocityAndHeadingScript = false;
      boolean cheatWithGroundHeightAtForFootstep = false;

      DRCFlatGroundWalkingTrack flatGroundWalkingTrack=new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup,
                                    useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, robotModel);
      flatGroundWalkingTrack.getDrcSimulation().start();
      flatGroundWalkingTrack.getDrcSimulation().simulate();
      SimulationConstructionSet scs = flatGroundWalkingTrack.getSimulationConstructionSet();
      
      scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk").setValueFromDouble(1.0);
      scs.getVariable("DesiredHeadingControlModule", "desiredHeadingDot").setValueFromDouble(0.1); // slightly left turn
      scs.getVariable("MomentumBasedControllerFactory","swingTime").setValueFromDouble(0.75); // slightly left turn
      scs.getVariable("MomentumBasedControllerFactory","transferTime").setValueFromDouble(0.25); // slightly left turn
      scs.getVariable("DesiredHeadingControlModule", "desiredHeadingDot").setValueFromDouble(0.1); // slightly left turn
      YoVariable<?> desiredVelocityX =  scs.getVariable("SimpleDesiredVelocityControlModule", "desiredVelocityX");
      

      for(double v=0.0; v<0.6; v+=0.1)
      {
              desiredVelocityX.setValueFromDouble(v); 
              ThreadTools.sleep(10000);
      }
      
      scs.stop();
      
   }
}