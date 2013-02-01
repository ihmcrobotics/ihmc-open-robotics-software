package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.DrivingHighLevelHumanoidFactory;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCRobotMomentumBasedControllerFactory;
import us.ihmc.darpaRoboticsChallenge.initialSetup.SquaredUpDRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;

import com.martiansoftware.jsap.JSAPException;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCDemo03
{
   private final HumanoidRobotSimulation<SDFRobot> drcSimulation;
   private final DRCDemoEnvironmentWithBoxAndSteeringWheel environment;

   public DRCDemo03(DRCRobotModel robotModel,
         DRCGuiInitialSetup guiInitialSetup, AutomaticSimulationRunner automaticSimulationRunner, double timePerRecordTick,
         int simulationDataBufferSize, String ipAddress, int portNumber)
   {
      DRCSCSInitialSetup scsInitialSetup;
      RobotInitialSetup<SDFRobot> robotInitialSetup = new SquaredUpDRCRobotInitialSetup();

      environment = new DRCDemoEnvironmentWithBoxAndSteeringWheel();
      scsInitialSetup = new DRCSCSInitialSetup(environment);
      scsInitialSetup.setSimulationDataBufferSize(simulationDataBufferSize);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(timePerRecordTick / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();


      DrivingHighLevelHumanoidFactory highLevelHumanoidControllerFactory = new DrivingHighLevelHumanoidFactory();

      ControllerFactory controllerFactory = new DRCRobotMomentumBasedControllerFactory(highLevelHumanoidControllerFactory, true);

      drcSimulation = DRCSimulationFactory.createSimulation(robotModel, controllerFactory, environment, robotInitialSetup, scsInitialSetup, guiInitialSetup);

      SimulationConstructionSet simulationConstructionSet = drcSimulation.getSimulationConstructionSet();

      // add other registries
      drcSimulation.addAdditionalDynamicGraphicObjectsListRegistries(dynamicGraphicObjectsListRegistry);

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
         String cameraName = "stereo_camera";
         if (robotModel == DRCRobotModel.ATLAS_NO_HANDS)
         {
            cameraName = "left_camera_sensor";
         }

         CameraConfiguration cameraConfiguration = new CameraConfiguration(cameraName);
         cameraConfiguration.setCameraMount(cameraName);

         

         // Decrease resolution to improve performance
         // TODO: Revert to full resolution images
//         SDFCamera camera = drcSimulation.getRobot().getCamera(cameraName);;
//         int width = camera.getWidth();
//         int height = camera.getHeight();
         
         drcSimulation.getSimulationConstructionSet().startStreamingVideoData(cameraConfiguration, DRCConfigParameters.VIDEOSETTINGS,
                 DRCConfigParameters.BG_VIDEO_SERVER_PORT_NUMBER);
      }
   }

   public static void main(String[] args) throws JSAPException
   {
      AutomaticSimulationRunner automaticSimulationRunner = null;

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup();

      double timePerRecordTick = 0.005;
      int simulationDataBufferSize = 16000;
      String ipAddress = null;
      int portNumber = -1;
      new DRCDemo03(DRCRobotModel.getDefaultRobotModel(), guiInitialSetup, automaticSimulationRunner, timePerRecordTick, simulationDataBufferSize, ipAddress,
                                     portNumber);
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }
}
