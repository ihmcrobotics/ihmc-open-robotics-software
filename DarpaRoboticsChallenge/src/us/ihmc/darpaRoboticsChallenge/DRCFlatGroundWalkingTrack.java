package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.FlatGroundWalkingHighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.terrain.TerrainType;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotMomentumBasedControllerFactory;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.SquaredUpDRCRobotInitialSetup;

import com.martiansoftware.jsap.JSAPException;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.inputdevices.MidiSliderBoard;

public class DRCFlatGroundWalkingTrack
{
   private final DRCSimulation drcSimulation;
   private final DRCDemo01Environment environment;

   public DRCFlatGroundWalkingTrack(DRCGuiInitialSetup guiInitialSetup, AutomaticSimulationRunner automaticSimulationRunner, double timePerRecordTick,
                   int simulationDataBufferSize, boolean doChestOrientationControl)
   {
      DRCSCSInitialSetup scsInitialSetup;
      DRCRobotInitialSetup drcRobotInitialSetup;

      drcRobotInitialSetup = new SquaredUpDRCRobotInitialSetup();
      
      
      
      scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT);
      scsInitialSetup.setSimulationDataBufferSize(simulationDataBufferSize);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(timePerRecordTick / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);



      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("adjustableParabolicTrajectoryDemoSimRegistry");

      double desiredCoMHeight = 0.8;
      double inPlaceWidth = 0.25;
      double maxStepLength = 0.35;
      double minStepWidth = 0.15;
      double maxStepWidth = 0.4;
      double stepPitch = 0.0;
      HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory = new FlatGroundWalkingHighLevelHumanoidControllerFactory(desiredCoMHeight,
                                                                                 inPlaceWidth, maxStepLength, minStepWidth, maxStepWidth, stepPitch);
      ControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(highLevelHumanoidControllerFactory);

      environment = new DRCDemo01Environment();

//      r2Simulation = new R2Simulation(environment, r2InitialSetup, sensorNoiseInitialSetup, controllerFactory, scsInitialSetup, guiInitialSetup);
      drcSimulation = new DRCSimulation(drcRobotInitialSetup, controllerFactory, scsInitialSetup, guiInitialSetup);

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

      new DRCFlatGroundWalkingTrack(guiInitialSetup, automaticSimulationRunner, 0.005, 16000, true);
   }


   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }

   public DRCDemo01Environment getEnvironment()
   {
      return environment;
   }
}
