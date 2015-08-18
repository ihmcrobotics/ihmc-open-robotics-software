package us.ihmc.valkyrie.simulation;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTrack;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.joystick.JoystickUpdater;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard;
import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard.MultisenseHeadingSubscriber;
import java.net.URI;


public class ValkyrieFlatGroundWalkingTrack
{
   public static void main(String[] args)
   {
      boolean USE_JOYSTICK_CONTROLLER = JoystickUpdater.isJoyStickConnected();
      
      DRCRobotModel robotModel = new ValkyrieRobotModel(false, false);
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);      

      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);
      
      double initialYaw = 0.0;
      DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      boolean useVelocityAndHeadingScript = !USE_JOYSTICK_CONTROLLER;;
      boolean cheatWithGroundHeightAtForFootstep = false;
      
      DRCFlatGroundWalkingTrack flatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup,
            useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, robotModel,
            WalkingProvider.VELOCITY_HEADING_COMPONENT);
      
      SimulationConstructionSet scs = flatGroundWalkingTrack.getSimulationConstructionSet();
     
		String namespace = "/multisense",nodeName = "/heading_listener", headingTopicName="/multisense/vector_to_target";

		final RosMainNode rosMainNode;	
		
      if (USE_JOYSTICK_CONTROLLER)
      {
    	URI rosUri = NetworkParameters.getROSURI();
    	rosMainNode = new RosMainNode(rosUri, namespace + nodeName);
    	new MultisenseHeadingSubscriber(rosMainNode, headingTopicName,scs.getRootRegistry());      	  		
  		rosMainNode.execute();  		
  		while(!rosMainNode.isStarted())
  		{
  		   ThreadTools.sleep(100);
  		}

  		 
         ValkyrieSliderBoard.setupJoyStickAndTreadmill(scs.getRootRegistry());
         flatGroundWalkingTrack.getDrcSimulation().start();
         flatGroundWalkingTrack.getDrcSimulation().simulate();
      }
   }

}
