package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ComponentBasedVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.robotSide.SideDependentList;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class DRCFlatGroundWalkingTrack
{
   private final DRCSimulationFactory drcSimulation;

   public DRCFlatGroundWalkingTrack(DRCRobotInitialSetup<SDFRobot> robotInitialSetup, DRCGuiInitialSetup guiInitialSetup, DRCSCSInitialSetup scsInitialSetup,
         boolean useVelocityAndHeadingScript, boolean cheatWithGroundHeightAtForFootstep, DRCRobotModel model)
   {
      WalkingControllerParameters walkingControlParameters = model.getWalkingControllerParameters();
      ArmControllerParameters armControllerParameters = model.getArmControllerParameters();

      //    scsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(model.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);

      WalkingControllerParameters walkingControllerParameters = model.getWalkingControllerParameters();
      DRCRobotContactPointParameters contactPointParameters = model.getContactPointParameters();
      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();

      SideDependentList<String> footForceSensorNames = model.getSensorInformation().getFeetForceSensorNames();

      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory,
            footForceSensorNames, walkingControllerParameters, armControllerParameters,
            HighLevelState.WALKING);
      
      HeightMap heightMapForCheating = null;
      if (cheatWithGroundHeightAtForFootstep)
      {
         heightMapForCheating = scsInitialSetup.getHeightMap();
      }

      VariousWalkingProviderFactory variousWalkingProviderFactory = new ComponentBasedVariousWalkingProviderFactory(useVelocityAndHeadingScript, heightMapForCheating, model.getControllerDT());
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
