package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxEndToEndTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

import java.io.IOException;
import java.io.InputStream;

public class ValkyrieKinematicsStreamingToolboxEndToEndTest extends KinematicsStreamingToolboxEndToEndTest
{
   private static final String RESOURCE_DIRECTORY = "us/ihmc/kinematicsStreamingToolboxLogs";

   @Override
   public DRCRobotModel newRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false);
   }

   @Test
   public void testSimpleArmMotions() throws IOException, SimulationExceededMaximumTimeException
   {
      String fileName = "20190926_102802_ValkyrieKinematicsStreamingToolbox_SimpleArmMotions.json";
      InputStream inputStream = loadInputStream(fileName);
      runTest(inputStream);
   }

   @Test
   public void testArmCollisions() throws IOException, SimulationExceededMaximumTimeException
   {
      String fileName = "20190926_101146_ValkyrieKinematicsStreamingToolbox_ArmCollisions.json";
      InputStream inputStream = loadInputStream(fileName);
      runTest(inputStream);
   }

   @Test
   public void testCrazyInputsLog() throws IOException, SimulationExceededMaximumTimeException
   {
      String fileName = "20190926_103042_ValkyrieKinematicsStreamingToolbox_CrazyInputs.json";
      InputStream inputStream = loadInputStream(fileName);
      runTest(inputStream);
   }

   private InputStream loadInputStream(String fileName)
   {
      String resourcePath = RESOURCE_DIRECTORY + "/" + fileName;
      return ValkyrieKinematicsStreamingToolboxEndToEndTest.class.getClassLoader().getResourceAsStream(resourcePath);
   }
}
