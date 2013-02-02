package us.ihmc.darpaRoboticsChallenge;

import static org.junit.Assert.fail;

import org.junit.After;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.GroundProfile;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.FlatGroundProfile;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class DRCFlatGroundWalkingTest
{
   private static final boolean ALWAYS_SHOW_GUI = false;
   private static final boolean KEEP_SCS_UP = false;

   private static final boolean CREATE_MOVIE = BambooTools.doMovieCreation();
   private static final boolean SHOW_GUI = ALWAYS_SHOW_GUI || CREATE_MOVIE;

   private BlockingSimulationRunner blockingSimulationRunner;
   
   @After
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }
   }

   @Test
   public void testDRC2OverShallowRamp() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double standingTimeDuration = 1.0;
      double walkingTimeDuration = 90.0;
      double desiredVelocityValue = 1.0;
      double desiredHeadingValue = 0.0;

      double minCenterOfMassHeight = 0.8;
      double maxCenterOfMassHeight = 1.5;

      boolean useVelocityAndHeadingScript = false;

      double rampXStart = -0.25;
      double rampYStart = -2.0;
      double rampXEnd = 10.0;
      double rampYEnd = 6.0;
      double rampHeight = 0.5;

      CombinedTerrainObject combinedTerrainObject = new CombinedTerrainObject("JustARamp");
      combinedTerrainObject.addRamp(rampXStart, rampYStart, rampXEnd, rampYEnd, rampHeight, YoAppearance.Red());
      combinedTerrainObject.addBox(rampXStart, rampYStart, rampXEnd, rampYEnd, -0.05, 0.0);

      SimulationConstructionSet scs = setupScs(combinedTerrainObject, useVelocityAndHeadingScript);
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(combinedTerrainObject.getLinkGraphics());
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("walk");
      DoubleYoVariable desiredSpeed = (DoubleYoVariable) scs.getVariable("desiredVelocityX");
      DoubleYoVariable desiredHeading = (DoubleYoVariable) scs.getVariable("desiredHeading");
//      DoubleYoVariable centerOfMassHeight = (DoubleYoVariable) scs.getVariable("ProcessedSensors.comPositionz");
      DoubleYoVariable comError = (DoubleYoVariable) scs.getVariable("positionError_comHeight");
      DoubleYoVariable leftFootHeight = (DoubleYoVariable) scs.getVariable("p_leftFootPositionZ");
      DoubleYoVariable rightFootHeight = (DoubleYoVariable) scs.getVariable("p_rightFootPositionZ");

      initiateMotion(standingTimeDuration, blockingSimulationRunner, walk);
      desiredSpeed.set(desiredVelocityValue);
      desiredHeading.set(desiredHeadingValue);

      double timeIncrement = 1.0;
      while (scs.getTime() - standingTimeDuration < walkingTimeDuration)
      {
         blockingSimulationRunner.simulateAndBlock(timeIncrement);
//         double adjustedCoM = centerOfMassHeight.getDoubleValue() - Math.min(leftFootHeight.getDoubleValue(), rightFootHeight.getDoubleValue());
         if (Math.abs(comError.getDoubleValue()) > 0.01)
         {
            fail("Math.abs(comError.getDoubleValue()");
         }
      }

      createMovie(scs);
   }

   @Test
   public void testDRCFlatGroundWalking() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double standingTimeDuration = 1.0;
      double walkingTimeDuration = 90.0;
      double epsilonHeading = Math.PI / 4.0;
      double minCenterOfMassHeight = 0.8;
      double maxCenterOfMassHeight = 1.5;

      boolean useVelocityAndHeadingScript = true;

      GroundProfile groundProfile = new FlatGroundProfile();

      SimulationConstructionSet scs = setupScs(groundProfile, useVelocityAndHeadingScript);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("walk");
//      DoubleYoVariable desiredHeading = (DoubleYoVariable) scs.getVariable("desiredHeading");
      DoubleYoVariable pelvisYaw = (DoubleYoVariable) scs.getVariable("q_yaw");
//    DoubleYoVariable centerOfMassHeight = (DoubleYoVariable) scs.getVariable("ProcessedSensors.comPositionz");
      DoubleYoVariable comError = (DoubleYoVariable) scs.getVariable("positionError_comHeight");
    
      initiateMotion(standingTimeDuration, blockingSimulationRunner, walk);

      ThreadTools.sleepForever();
      
      double timeIncrement = 1.0;

      while (scs.getTime() - standingTimeDuration < walkingTimeDuration)
      {
         blockingSimulationRunner.simulateAndBlock(timeIncrement);
         
         //TODO: Put test for heading back in here.
//         if (!MathTools.epsilonEquals(desiredHeading.getDoubleValue(), pelvisYaw.getDoubleValue(), epsilonHeading))
//         {
//            fail("Desired Heading too large of error. desiredHeading.getDoubleValue()");
//         }
         
         if (Math.abs(comError.getDoubleValue()) > 0.01)
         {
            fail("Math.abs(comError.getDoubleValue()) > 0.01");
         }
      }

      createMovie(scs);
      BambooTools.reportTestFinishedMessage();

   }

   private void initiateMotion(double standingTimeDuration, BlockingSimulationRunner runner, BooleanYoVariable walk)
           throws SimulationExceededMaximumTimeException
   {
      walk.set(false);
      runner.simulateAndBlock(standingTimeDuration);
      walk.set(true);
   }

   private void createMovie(SimulationConstructionSet scs)
   {
      if (CREATE_MOVIE)
      {
         BambooTools.createMovieAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(scs);
      }

      if (KEEP_SCS_UP)
      {
         BambooTools.sleepForever();
      }
   }

   private SimulationConstructionSet setupScs(GroundProfile groundProfile, boolean useVelocityAndHeadingScript)
   {
      AutomaticSimulationRunner automaticSimulationRunner = null;
      DRCGuiInitialSetup guiInitialSetup = createGUIInitialSetup();
   
      DRCRobotModel robotModel = DRCRobotModel.getDefaultRobotModel();
      double timePerRecordTick = 0.005;
      int simulationDataBufferSize = 16000;
      boolean doChestOrientationControl = true;
      
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile);
      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack =  new DRCFlatGroundWalkingTrack(robotModel, guiInitialSetup, scsInitialSetup, automaticSimulationRunner, timePerRecordTick, simulationDataBufferSize, doChestOrientationControl);
      
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();
      
      setupCameraForUnitTest(scs);

      return scs;
   }

   private DRCGuiInitialSetup createGUIInitialSetup()
   {
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup();
      guiInitialSetup.setIsGuiShown(SHOW_GUI);
      
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
