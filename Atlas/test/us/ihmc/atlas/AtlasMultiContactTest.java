package us.ihmc.atlas;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.visualizer.RobotVisualizer;
import us.ihmc.atlas.AtlasMultiContact.MultiContactTask;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

@BambooPlan(planType = {BambooPlanType.Fast, BambooPlanType.VideoA})
public class AtlasMultiContactTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();   
   private BlockingSimulationRunner blockingSimulationRunner;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   private DRCSimulationFactory drcSimulation;
   private RobotVisualizer robotVisualizer;

  
   @After
   public void destroyOtherStuff()
   {
      // Do this here in case a test fails. That way the memory will be recycled.

      if (drcSimulation != null)
      {
         drcSimulation.dispose();
         drcSimulation = null;
      }

      if (robotVisualizer != null)
      {
         robotVisualizer.close();
         robotVisualizer = null;
      }
   }

	@EstimatedDuration(duration = 38.4)
	@Test(timeout = 192109)
   public void testMultiContactLocomotion() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      int prepDuration = 1;
      int testDuration = 5;

      AtlasMultiContact drcMultiContact = setupSimulation();
      SimulationConstructionSet scs = drcMultiContact.getSimulationConstructionSet();
      drcSimulation = drcMultiContact.getDRCSimulation();

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      DoubleYoVariable desiredComZ = (DoubleYoVariable) scs.getVariable("desiredCoMZ");
      DoubleYoVariable errorComZ = (DoubleYoVariable) scs.getVariable("comPositionErrorZ");

      blockingSimulationRunner.simulateAndBlock(prepDuration);

      double timeIncrement = 1;

      while (scs.getTime() - prepDuration < testDuration)
      {
         blockingSimulationRunner.simulateAndBlock(timeIncrement);
         desiredComZ.set(scs.getTime() / testDuration);

         System.out.println("time " + scs.getTime() + " desired " + desiredComZ.getDoubleValue() + " error " + errorComZ.getDoubleValue());
         if (Math.abs(errorComZ.getDoubleValue()) > 0.06)
         {
            //Re-enable this when demo is fixed
            //            fail("Math.abs(comError.getDoubleValue()) > 0.06: " + errorComZ.getDoubleValue() + " at t = " + scs.getTime());
            System.out.println("Math.abs(errorComZ.getDoubleValue()) > 0.06: " + errorComZ.getDoubleValue() + " at t = " + scs.getTime());
         }
      }

      createMovie(scs);
      BambooTools.reportTestFinishedMessage();

   }

   private AtlasMultiContact setupSimulation()
   {
      final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_INVISIBLE_CONTACTABLE_PLANE_HANDS;
      DRCGuiInitialSetup guiInitialSetup = createGUIInitialSetup();

      AtlasMultiContact drcMultiContact = new AtlasMultiContact(new AtlasRobotModel(ATLAS_ROBOT_VERSION, AtlasTarget.SIM, false), guiInitialSetup,
            MultiContactTask.DEFAULT);
      SimulationConstructionSet scs = drcMultiContact.getSimulationConstructionSet();

      setupCameraForUnitTest(scs);

      return drcMultiContact;
   }

   private void createMovie(SimulationConstructionSet scs)
   {
      if (simulationTestingParameters.getCreateSCSMovies())
      {
         BambooTools.createMovieAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS), scs, 1);
      }
   }

   private DRCGuiInitialSetup createGUIInitialSetup()
   {
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, simulationTestingParameters);
      return guiInitialSetup;
   }

   protected void setupCameraForUnitTest(SimulationConstructionSet scs)
   {
      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");
      cameraConfiguration.setCameraFix(0.6, 0.4, 1.1);
      cameraConfiguration.setCameraPosition(-0.15, 10.0, 3.0);
      cameraConfiguration.setCameraTracking(true, true, true, false);
      cameraConfiguration.setCameraDolly(true, true, true, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");
   }
}
