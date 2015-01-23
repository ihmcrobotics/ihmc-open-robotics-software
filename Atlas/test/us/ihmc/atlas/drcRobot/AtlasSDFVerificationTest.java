package us.ihmc.atlas.drcRobot;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.NothingChangedVerifier;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

/**
 * Created by dstephen on 2/11/14.
 */
public class AtlasSDFVerificationTest
{
   private static final boolean KEEP_SCS_UP = false;
   private static final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.DRC_NO_HANDS;

   private static final boolean CREATE_MOVIE = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();

   private static final double SIM_DURATION = 5.0;

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

   @Test(timeout=300000)
   public void testSimpleLegSwing() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage();

      DRCRobotModel selectedModel = new AtlasRobotModel(ATLAS_ROBOT_VERSION, AtlasRobotModel.AtlasTarget.SIM, false);
      SDFRobot sdfRobot = selectedModel.createSdfRobot(true);

      pinRobotInAir(sdfRobot);

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);

      NothingChangedVerifier nothingChangedVerifier = setupNothingChangedVerifier(scs);

      simulate(scs);

      if (checkNothingChanged)
      {
         checkNothingChanged(nothingChangedVerifier);
      }

      if (CREATE_MOVIE)
      {
         BambooTools.createMovieAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS), scs, 1);
      }

      BambooTools.reportTestFinishedMessage();

//    fail();
   }

   private void pinRobotInAir(SDFRobot sdfRobot)
   {
      sdfRobot.setPositionInWorld(new Vector3d(sdfRobot.getPositionInWorld().x, sdfRobot.getPositionInWorld().y, sdfRobot.getPositionInWorld().z + 0.5));
      ExternalForcePoint fp = new ExternalForcePoint("gravityCompensation", sdfRobot.getRobotsYoVariableRegistry());
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

   private void simulate(SimulationConstructionSet scs) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException, IOException
   {
      scs.startOnAThread();

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);
      double timeIncrement = 1.0;
      while (scs.getTime() < SIM_DURATION)
      {
         blockingSimulationRunner.simulateAndBlock(timeIncrement);
      }

      HashMap<String, Double> yoVariables = getDoubleYoVariables(scs.getRobots()[0]);

      boolean result = verifyDump(yoVariables);

      writeLatestDump(yoVariables);

      assertTrue(result);
   }

   private HashMap<String, Double> getDoubleYoVariables(Robot robot)
   {
      HashMap<String, Double> yoVariables = new HashMap<>();

      for (YoVariable var : robot.getAllVariables())
      {
         if (var instanceof DoubleYoVariable)
         {
            yoVariables.put(var.getName(), ((DoubleYoVariable) var).getDoubleValue());
         }
      }

      return yoVariables;
   }

   private boolean verifyDump(HashMap<String, Double> yoVariables) throws IOException
   {
      File dumpFile = new File("sdfVerify.dump");

      if (dumpFile.exists())
      {
         BufferedReader reader = new BufferedReader(new FileReader(dumpFile));
         String line;

         while ((line = reader.readLine()) != null)
         {
            String name = line.substring(0, line.indexOf(":"));
            double value = Double.parseDouble(line.substring(line.indexOf(":") + 1));

            if (hasVariableChangedSignificantly(yoVariables, name, value))
            {
               return false;
            }
         }

         return true;
      }
      else
      {
         System.err.println("Couldn't find variable dump in project root, skipping verification step. Try running test again.");

         return true;
      }
   }

   private boolean hasVariableChangedSignificantly(HashMap<String, Double> yoVariables, String name, double value)
   {
      if (yoVariables.containsKey(name))
      {

         if (!MathTools.epsilonEquals(yoVariables.get(name).doubleValue(), value, 1e-10))
         {
            System.err.println("Values for " + name + "out of range");
            System.err.println("Current: " + yoVariables.get(name).doubleValue());
            System.err.println("Dumped: " + value);

            return true;
         }
      }
      else
      {
         System.out.println("No variable named: " + name);

         return true;
      }

      return false;
   }

   private void writeLatestDump(HashMap<String, Double> yoVariables) throws FileNotFoundException
   {
      PrintWriter writer = new PrintWriter(new FileOutputStream("sdfVerify.dump"));
      for (String name : yoVariables.keySet())
      {
         String line = name + ":" + yoVariables.get(name).doubleValue();
         writer.println(line);
      }
      writer.flush();
      writer.close();
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
