package us.ihmc.atlas.drcRobot;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.simulationTesting.NothingChangedVerifier;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Created by dstephen on 2/11/14.
 */
public class AtlasSDFVerificationTest
{
   private SimulationTestingParameters simulationTestingParameters;   
   private BlockingSimulationRunner blockingSimulationRunner;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
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

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   private static final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final double SIM_DURATION = 5.0;

   /**
    * February 16, 2015. This passes locally, but not on Bamboo. It does some crazy file writing, but in the wrong order seemingly. Someone please fix...
    * 
    * @throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
    * @throws IOException
    * @throws ControllerFailureException
    */
	@Disabled
	@Test
   public void testSimpleLegSwing() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException, IOException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel selectedModel = new AtlasRobotModel(ATLAS_ROBOT_VERSION, RobotTarget.SCS, false);
      FloatingRootJointRobot sdfRobot = selectedModel.createHumanoidFloatingRootJointRobot(true);

      pinRobotInAir(sdfRobot);

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);

      NothingChangedVerifier nothingChangedVerifier = setupNothingChangedVerifier(scs);

      simulate(scs);

      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
      {
         checkNothingChanged(nothingChangedVerifier);
      }

      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS), scs, 1);
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

//    fail();
   }

   private void pinRobotInAir(FloatingRootJointRobot sdfRobot)
   {
      sdfRobot.setPositionInWorld(new Vector3D(sdfRobot.getPositionInWorld().getX(), sdfRobot.getPositionInWorld().getY(), sdfRobot.getPositionInWorld().getZ() + 0.5));
      ExternalForcePoint fp = new ExternalForcePoint("gravityCompensation", sdfRobot.getRobotsYoVariableRegistry());
      fp.setForce(0, 0, -sdfRobot.getGravityZ() * sdfRobot.computeCenterOfMass(new Point3D()) / 2);

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

   private void simulate(SimulationConstructionSet scs) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException, IOException, ControllerFailureException
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

      for (YoVariable<?> var : robot.getAllVariables())
      {
         if (var instanceof YoDouble)
         {
            yoVariables.put(var.getName(), ((YoDouble) var).getDoubleValue());
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
            String name = line.substring(0, line.indexOf(':'));
            double value = Double.parseDouble(line.substring(line.indexOf(':') + 1));

            if (hasVariableChangedSignificantly(yoVariables, name, value))
            {
               reader.close();
               return false;
            }
         }

         reader.close();
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
      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
      {
         nothingChangedVerifier = new NothingChangedVerifier("AtlasSDFVerificationTest.SimpleLegSwing", scs);
      }

      return nothingChangedVerifier;
   }
}
