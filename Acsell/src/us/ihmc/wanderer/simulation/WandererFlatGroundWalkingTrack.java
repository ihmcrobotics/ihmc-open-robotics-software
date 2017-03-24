package us.ihmc.wanderer.simulation;

import java.io.IOException;

import net.java.games.input.Component;
import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionsettools.joystick.BooleanYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionsettools.joystick.DoubleYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.wanderer.parameters.WandererRobotModel;

public class WandererFlatGroundWalkingTrack
{
   public static void main(String[] args)
   {
      boolean USE_JOYSTICK_CONTROLLER = Joystick.isAJoystickConnectedToSystem();

      WandererRobotModel robotModel = new WandererRobotModel(false, false);
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);


      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      double initialYaw = 0.3;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);
      boolean useVelocityAndHeadingScript = !USE_JOYSTICK_CONTROLLER;
      boolean cheatWithGroundHeightAtForFootstep = false;

      HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters = new HeadingAndVelocityEvaluationScriptParameters();
      DRCFlatGroundWalkingTrack flatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup,
                                                            useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, robotModel,
                                                            WalkingProvider.VELOCITY_HEADING_COMPONENT, walkingScriptParameters);


      SimulationConstructionSet scs = flatGroundWalkingTrack.getSimulationConstructionSet();
      if (USE_JOYSTICK_CONTROLLER)
      {
         setupJoyStick(scs);
         flatGroundWalkingTrack.getAvatarSimulation().start();
         flatGroundWalkingTrack.getAvatarSimulation().simulate();
      }
   }


   public static void setupJoyStick(YoVariableHolder registry)
   {
      Joystick joystickUpdater;
      try
      {
         joystickUpdater = new Joystick();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return;
      }
      
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
      joystickUpdater.addJoystickEventListener(new DoubleYoVariableJoystickEventListener(desiredVelocityX, joystickUpdater.findComponent(Component.Identifier.Axis.Y),
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
      joystickUpdater.addJoystickEventListener(new DoubleYoVariableJoystickEventListener(desiredVelocityY, joystickUpdater.findComponent(Component.Identifier.Axis.X),
    		  -0.1+desiredVelocityY_Bias, 0.1+desiredVelocityY_Bias, deadZone, true));

      DoubleYoVariable desiredHeadingDot = (DoubleYoVariable) registry.getVariable("RateBasedDesiredHeadingControlModule", "desiredHeadingDot");
      desiredHeadingDot.set(desiredHeadingDot_Bias);
      joystickUpdater.addJoystickEventListener(new DoubleYoVariableJoystickEventListener(desiredHeadingDot, joystickUpdater.findComponent(Component.Identifier.Axis.RZ),
    		  -0.1+desiredHeadingDot_Bias, 0.1+desiredHeadingDot_Bias, deadZone/2.0, true));
      
      BooleanYoVariable walk = (BooleanYoVariable) registry.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper","walk");
      joystickUpdater.addJoystickEventListener(new BooleanYoVariableJoystickEventListener(walk, joystickUpdater.findComponent(Component.Identifier.Button.TRIGGER), true));
   }
}
