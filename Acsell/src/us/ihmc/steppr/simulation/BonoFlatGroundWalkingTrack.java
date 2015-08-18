package us.ihmc.steppr.simulation;

import net.java.games.input.Component;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.steppr.parameters.BonoRobotModel;
import us.ihmc.acsell.treadmill.TreadmillJoystickEventListener;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTrack;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.joystick.BooleanYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionset.joystick.DoubleYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionset.joystick.JoyStickNotFoundException;
import us.ihmc.simulationconstructionset.joystick.JoystickUpdater;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.steppr.hardware.visualization.StepprStandPrepSliderboard;
import us.ihmc.yoUtilities.dataStructure.YoVariableHolder;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

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
      DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);
      boolean useVelocityAndHeadingScript = !USE_JOYSTICK_CONTROLLER;
      boolean cheatWithGroundHeightAtForFootstep = false;

      DRCFlatGroundWalkingTrack flatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup,
                                                            useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, robotModel,
                                                            WalkingProvider.VELOCITY_HEADING_COMPONENT);


      SimulationConstructionSet scs = flatGroundWalkingTrack.getSimulationConstructionSet();
      if (USE_JOYSTICK_CONTROLLER)
      {
         setupJoyStick(scs);
         flatGroundWalkingTrack.getDrcSimulation().start();
         flatGroundWalkingTrack.getDrcSimulation().simulate();
      }
   }


   public static void setupJoyStick(YoVariableHolder registry)
   {
	  
	  final JoystickUpdater joystickUpdater;
	  try
	  {
		   joystickUpdater = new JoystickUpdater();
	  }
      catch (JoyStickNotFoundException ex)
      {
    		  System.err.println("Joystick not found. Proceeding without joystick");
    		  return;
      }
      Thread thread = new Thread(joystickUpdater);
      thread.start();

      
      double deadZone = 0.02;
      double desiredVelocityX_Bias = 0.0;
      double desiredVelocityY_Bias = 0.0;
      double desiredHeadingDot_Bias = 0.0;
      final double maxDesiredVelocityX = 0.25;
      final double minVelocityX = -0.10;
      
      
      DoubleYoVariable desiredVelocityX = (DoubleYoVariable) registry.getVariable("ManualDesiredVelocityControlModule", "desiredVelocityX");
      if(desiredVelocityX==null || joystickUpdater==null)
         return;

      desiredVelocityX.set(desiredVelocityX_Bias);
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredVelocityX, joystickUpdater.findComponent(Component.Identifier.Axis.Y),
    		  -maxDesiredVelocityX+desiredVelocityX_Bias, maxDesiredVelocityX+desiredVelocityX_Bias, deadZone, true));
/*      desiredVelocityX.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
         	if (v.getValueAsDouble() < minVelocityX)
            	 v.setValueFromDouble(minVelocityX, false);
         }
      });
*/      
      DoubleYoVariable desiredVelocityY = (DoubleYoVariable) registry.getVariable("ManualDesiredVelocityControlModule", "desiredVelocityY");
      desiredVelocityY.set(desiredVelocityY_Bias);
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredVelocityY, joystickUpdater.findComponent(Component.Identifier.Axis.X),
    		  -0.1+desiredVelocityY_Bias, 0.1+desiredVelocityY_Bias, deadZone, true));

      DoubleYoVariable desiredHeadingDot = (DoubleYoVariable) registry.getVariable("RateBasedDesiredHeadingControlModule", "desiredHeadingDot");
      desiredHeadingDot.set(desiredHeadingDot_Bias);
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredHeadingDot, joystickUpdater.findComponent(Component.Identifier.Axis.RZ),
    		  -0.1+desiredHeadingDot_Bias, 0.1+desiredHeadingDot_Bias, deadZone/2.0, true));
      
      joystickUpdater.listComponents();
      
      BooleanYoVariable walk = (BooleanYoVariable) registry.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper","walk");
      joystickUpdater.addListener(new BooleanYoVariableJoystickEventListener(walk, joystickUpdater.findComponent(Component.Identifier.Button.TRIGGER), true));
      
   }
   
}
