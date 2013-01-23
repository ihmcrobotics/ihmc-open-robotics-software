package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFCamera;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.FlatGroundWalkingHighLevelHumanoidControllerFactory;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotMomentumBasedControllerFactory;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.SquaredUpDRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;

import com.martiansoftware.jsap.JSAPException;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.inputdevices.MidiSliderBoard;

public class DRCFlatGroundWalkingTrack
{
   private final DRCSimulation drcSimulation;
   private final DRCDemo01NavigationEnvironment environment;

   public DRCFlatGroundWalkingTrack(DRCGuiInitialSetup guiInitialSetup, AutomaticSimulationRunner automaticSimulationRunner, double timePerRecordTick,
                                    int simulationDataBufferSize, boolean doChestOrientationControl, String ipAddress, int portNumber)
   {
      DRCSCSInitialSetup scsInitialSetup;
      DRCRobotInitialSetup drcRobotInitialSetup;
      WalkingControllerParameters drcRobotParameters = new DRCRobotWalkingControllerParameters();

      drcRobotInitialSetup = new SquaredUpDRCRobotInitialSetup();

      environment = new DRCDemo01NavigationEnvironment();
      scsInitialSetup = new DRCSCSInitialSetup(environment);
      scsInitialSetup.setSimulationDataBufferSize(simulationDataBufferSize);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(timePerRecordTick / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);



      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("adjustableParabolicTrajectoryDemoSimRegistry");

      double inPlaceWidth = 0.25;
      double maxStepLength = 0.35;
      double minStepWidth = 0.15;
      double maxStepWidth = 0.4;
      double stepPitch = 0.0;
      FlatGroundWalkingHighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory = new FlatGroundWalkingHighLevelHumanoidControllerFactory(drcRobotParameters,
            inPlaceWidth, maxStepLength, minStepWidth, maxStepWidth, stepPitch);
      highLevelHumanoidControllerFactory.setupForNetworkedFootstepProvider(DRCConfigParameters.OPERATOR_INTERFACE_IP_ADDRESS, portNumber);
      ControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(highLevelHumanoidControllerFactory, true);

      

//    r2Simulation = new R2Simulation(environment, r2InitialSetup, sensorNoiseInitialSetup, controllerFactory, scsInitialSetup, guiInitialSetup);
      drcSimulation = new DRCSimulation(environment, drcRobotInitialSetup, controllerFactory, scsInitialSetup, guiInitialSetup);

      SimulationConstructionSet simulationConstructionSet = drcSimulation.getSimulationConstructionSet();
      MidiSliderBoard sliderBoard = new MidiSliderBoard(simulationConstructionSet);
      int i = 1;

      // TODO: get these from CommonAvatarUserInterface once it exists:
      sliderBoard.setSlider(i++, "desiredICPParameterX", getSimulationConstructionSet(), 0.0, 1.0);
      sliderBoard.setSlider(i++, "desiredICPParameterY", getSimulationConstructionSet(), 0.0, 1.0);
      sliderBoard.setSlider(i++, "desiredHeadingFinal", getSimulationConstructionSet(), Math.toRadians(-30.0), Math.toRadians(30.0));
      sliderBoard.setSlider(i++, "desiredPelvisPitch", getSimulationConstructionSet(), Math.toRadians(-20.0), Math.toRadians(20.0));
      sliderBoard.setSlider(i++, "desiredPelvisRoll", getSimulationConstructionSet(), Math.toRadians(-20.0), Math.toRadians(20.0));
      sliderBoard.setSlider(i++, "desiredCenterOfMassHeightFinal", getSimulationConstructionSet(), 0.42, 1.5);

      setUpJoyStick(getSimulationConstructionSet());

      // add other registries
      drcSimulation.addAdditionalDynamicGraphicObjectsListRegistries(dynamicGraphicObjectsListRegistry);
      drcSimulation.addAdditionalYoVariableRegistriesToSCS(registry);

      simulationConstructionSet.setCameraPosition(6.0, -2.0, 4.5);
      simulationConstructionSet.setCameraFix(-0.44, -0.17, 0.75);

      if (automaticSimulationRunner != null)
      {
         drcSimulation.start(automaticSimulationRunner);
      }
      else
      {
         drcSimulation.start(null);
      }

      if (DRCConfigParameters.STREAM_VIDEO)
      {
         System.out.println("Streaming SCS Video");
         
         String cameraName = "left_camera_sensor";
         CameraConfiguration videoConfiguration = new CameraConfiguration(cameraName);
         videoConfiguration.setCameraMount(cameraName);
         
         SDFCamera camera = drcSimulation.getRobot().getCamera(cameraName);
         int width = camera.getWidth();
         int height = camera.getHeight();
         
         // Decrease resolution to improve performance
         // TODO: Revert to full resolution images
         width = 640;
         height = 480;
         
         drcSimulation.getSimulationConstructionSet().startStreamingVideoData(videoConfiguration, width, height, DRCConfigParameters.BG_VIDEO_SERVER_PORT_NUMBER);
      }
   }

   private void setUpJoyStick(SimulationConstructionSet simulationConstructionSet)
   {
      try
      {
         new DRCJoystickController(simulationConstructionSet);
      }
      catch (RuntimeException e)
      {
         System.out.println("Could not connect to joystick");
      }
   }

   public static void main(String[] args) throws JSAPException
   {
      AutomaticSimulationRunner automaticSimulationRunner = null;

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup();

      double timePerRecordTick = 0.005;
      int simulationDataBufferSize = 16000;
      boolean doChestOrientationControl = true;
      String ipAddress = null;
      int portNumber = -1;
      new DRCFlatGroundWalkingTrack(guiInitialSetup, automaticSimulationRunner, timePerRecordTick, simulationDataBufferSize, doChestOrientationControl,
                                    ipAddress, portNumber);
   }


   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }

   public DRCDemo01NavigationEnvironment getEnvironment()
   {
      return environment;
   }
}
