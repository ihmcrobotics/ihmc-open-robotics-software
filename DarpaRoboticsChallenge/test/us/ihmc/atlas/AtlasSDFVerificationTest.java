package us.ihmc.atlas;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.time.GlobalTimer;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import com.yobotics.simulationconstructionset.util.simulationTesting.NothingChangedVerifier;
import org.junit.*;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;

import static org.junit.Assert.assertFalse;

/**
 * Created by dstephen on 2/11/14.
 */
public class AtlasSDFVerificationTest
{
   private static final boolean KEEP_SCS_UP = false;
   private static final boolean SIMULATE_FOREVER = false;

   private static final boolean CREATE_MOVIE = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingjChanged();

   private static final double SIM_DURATION = 15.0;

   private BlockingSimulationRunner blockingSimulationRunner;

   @BeforeClass
   public static void setUpBeforeClass()
   {
   }

   @AfterClass
   public static void tearDownAfterClass()
   {
   }

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void tearDown()
   {
      if (KEEP_SCS_UP)
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }

      GlobalTimer.clearTimers();
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");

   }

   @Test
   public void testSimpleLegSwing() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCRobotModel.ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS, false);
      JaxbSDFLoader loader = DRCRobotSDFLoader.loadDRCRobot(jointMap, false);
      SDFRobot sdfRobot = loader.createRobot(jointMap, true);

      pinRobotInAir(jointMap, sdfRobot);

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);

      NothingChangedVerifier nothingChangedVerifier = setupNothingChangedVerifier(scs);

      simulate(scs);

      if (checkNothingChanged)
      {
         checkNothingChanged(nothingChangedVerifier);
      }

      if (CREATE_MOVIE)
      {
         BambooTools.createMovieAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(scs, 1);
      }

      BambooTools.reportTestFinishedMessage();

//    fail();
   }

   private void pinRobotInAir(DRCRobotJointMap jointMap, SDFRobot sdfRobot)
   {
      sdfRobot.setPositionInWorld(new Vector3d(sdfRobot.getPositionInWorld().x, sdfRobot.getPositionInWorld().y, sdfRobot.getPositionInWorld().z + 0.5));
      ExternalForcePoint fp = new ExternalForcePoint("gravityCompensation", sdfRobot);
      fp.setForce(0, 0, -sdfRobot.getGravityZ() * sdfRobot.computeCenterOfMass(new Point3d()) / 2);

      sdfRobot.getJoint("l_arm_shx").addExternalForcePoint(fp);
      sdfRobot.getJoint("r_arm_shx").addExternalForcePoint(fp);
   }


   private void checkNothingChanged(NothingChangedVerifier nothingChangedVerifier)
   {
      ArrayList<String> stringsToIgnore = new ArrayList<>();
      stringsToIgnore.add("nano");
      stringsToIgnore.add("milli");
      stringsToIgnore.add("Timer");

      boolean writeNewBaseFile = nothingChangedVerifier.getWriteNewBaseFile();

      double maxPercentDifference = 0.001;
      nothingChangedVerifier.verifySameResultsAsPreviously(maxPercentDifference, stringsToIgnore);
      assertFalse("Had to write new base file. On next run nothing should change", writeNewBaseFile);
   }

   private void simulate(SimulationConstructionSet scs) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      scs.startOnAThread();

      if (SIMULATE_FOREVER)
      {
         ThreadTools.sleepForever();
      }
      else
      {
         blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);
         double timeIncrement = 1.0;
         while (scs.getTime() < SIM_DURATION)
         {
            blockingSimulationRunner.simulateAndBlock(timeIncrement);
         }
      }
   }

   private NothingChangedVerifier setupNothingChangedVerifier(SimulationConstructionSet scs)
   {
      NothingChangedVerifier nothingChangedVerifier = null;
      if (checkNothingChanged)
      {
         nothingChangedVerifier = new NothingChangedVerifier("AtlasSDFVerificationTest.SimpleLegSwing", scs);
      }

      return nothingChangedVerifier;
   }
}
