package us.ihmc.valkyrie.simulation;

import java.net.URI;

import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard;
import us.ihmc.valkyrie.controllers.ValkyrieSliderBoard.MultisenseHeadingSubscriber;


public class ValkyrieFlatGroundWalkingTrack
{
   public static void main(String[] args)
   {
//      boolean USE_JOYSTICK_CONTROLLER = JoystickUpdater.isJoyStickConnected();
      boolean USE_JOYSTICK_CONTROLLER = false;

      DRCRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);      

      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);
      
      double initialYaw = 0.0;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      boolean useVelocityAndHeadingScript = !USE_JOYSTICK_CONTROLLER;;
      boolean cheatWithGroundHeightAtForFootstep = false;
      
      HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters = new HeadingAndVelocityEvaluationScriptParameters();
      DRCFlatGroundWalkingTrack flatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup,
            useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, robotModel,
            WalkingProvider.VELOCITY_HEADING_COMPONENT, walkingScriptParameters);
      
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
         flatGroundWalkingTrack.getAvatarSimulation().start();
         flatGroundWalkingTrack.getAvatarSimulation().simulate();
      }
   }

}
