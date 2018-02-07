package us.ihmc.atlas.FlightPhaseTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

/**
 * Test fails as the high level humanoid controller manager does not have a flight state. 
 * Defaults to walking which results in ICP based control that fails
 * @author apoorv
 */
public class AtlasFlightPhaseControllerCoreTest
{
   @Test
   public void testControllerCoreWithoutContacts()
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      SimulationTestingParameters simulationTestParameters = new SimulationTestingParameters();
      simulationTestParameters.setKeepSCSUp(true);
      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      DRCSimulationTestHelper simulationTestHelper = new DRCSimulationTestHelper(simulationTestParameters, robotModel, flatGroundEnvironment);
      OffsetAndYawRobotInitialSetup startingLocation = new OffsetAndYawRobotInitialSetup(0.0, 0.0, 1.0);
      simulationTestHelper.setStartingLocation(startingLocation);
      simulationTestHelper.createSimulation(getClass().getName());
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      simulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
      try
      {
         simulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
         ThreadTools.sleepForever();
      }
      catch (Exception e)
      {
         PrintTools.error("Caught exception on creating sim");
      }
   }
}
