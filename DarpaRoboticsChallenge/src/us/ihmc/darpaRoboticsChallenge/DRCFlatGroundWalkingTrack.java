package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ComponentBasedVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.YoVariableVariousWalkingProviderFactory;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCRobotContactPointParameters;

public class DRCFlatGroundWalkingTrack
{
   private static final boolean CREATE_YOVARIABLE_WALKING_PROVIDERS = false; // Should always be committed as false.

   private final DRCSimulationFactory drcSimulation;

   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<SDFRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
         boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model)
   {
      ArmControllerParameters armControllerParameters = model.getArmControllerParameters();

      //    scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(model.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      WalkingControllerParameters walkingControllerParameters = model.getWalkingControllerParameters();
      DRCRobotContactPointParameters contactPointParameters = model.getContactPointParameters();
      CapturePointPlannerParameters capturePointPlannerParameters = model.getCapturePointPlannerParameters();
      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();

      SideDependentList<String> footForceSensorNames = model.getSensorInformation().getFeetForceSensorNames();

      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory,
            footForceSensorNames, walkingControllerParameters, armControllerParameters, capturePointPlannerParameters, 
            HighLevelState.WALKING);
      
      HeightMap heightMapForCheating = null;
      if (cheatWithGroundHeightAtForFootstep)
      {
         heightMapForCheating = scsInitialSetup.getHeightMap();
      }

      VariousWalkingProviderFactory variousWalkingProviderFactory;
      if (CREATE_YOVARIABLE_WALKING_PROVIDERS)
         variousWalkingProviderFactory = new YoVariableVariousWalkingProviderFactory();
      else         
         variousWalkingProviderFactory = new ComponentBasedVariousWalkingProviderFactory(useVelocityAndHeadingScript, heightMapForCheating, model.getControllerDT());

      controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);
      
      drcSimulation = new DRCSimulationFactory(model, controllerFactory, null, robotInitialSetup, scsInitialSetup, guiInitialSetup, null);

      drcSimulation.start();
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return drcSimulation.getSimulationConstructionSet();
   }

   public DRCSimulationFactory getDrcSimulation()
   {
      return drcSimulation;
   }
}
