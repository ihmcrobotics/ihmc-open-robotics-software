package us.ihmc.acsell.simulation;

import net.java.games.input.Component;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTrack;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.joystick.DoubleYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionset.joystick.JoystickUpdater;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.yoUtilities.dataStructure.YoVariableHolder;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class BonoFlatGroundWalkingTrack
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
      
      

      SimulationConstructionSet scs = flatGroundWalkingTrack.getSimulationConstructionSet();
      setupJoyStick(scs);
   }
   
   private static void setupJoyStick(YoVariableHolder registry)
   {
      final JoystickUpdater joystickUpdater = new JoystickUpdater();
      Thread thread = new Thread(joystickUpdater);
      thread.start();

      double deadZone = 0.02;

      DoubleYoVariable desiredVelocityX = (DoubleYoVariable) registry.getVariable("ManualDesiredVelocityControlModule", "desiredVelocityX");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredVelocityX, joystickUpdater.findComponent(Component.Identifier.Axis.Y), -0.3,
            0.3, deadZone, true));
      final double minVelocityX = -0.1;
      desiredVelocityX.addVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (v.getValueAsDouble() < minVelocityX)
               v.setValueFromDouble(minVelocityX, false);
         }
      });

      DoubleYoVariable desiredVelocityY = (DoubleYoVariable) registry.getVariable("ManualDesiredVelocityControlModule", "desiredVelocityY");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredVelocityY, joystickUpdater.findComponent(Component.Identifier.Axis.X), -0.1,
            0.1, deadZone, true));

      DoubleYoVariable desiredHeadingDot = (DoubleYoVariable) registry.getVariable("RateBasedDesiredHeadingControlModule", "desiredHeadingDot");
      joystickUpdater.addListener(new DoubleYoVariableJoystickEventListener(desiredHeadingDot, joystickUpdater.findComponent(Component.Identifier.Axis.RZ),
            -0.1, 0.1, deadZone, true));

   }
}