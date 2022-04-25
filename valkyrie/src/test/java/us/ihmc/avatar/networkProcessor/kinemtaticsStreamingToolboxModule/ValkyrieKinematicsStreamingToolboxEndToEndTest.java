package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import java.io.IOException;
import java.io.InputStream;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxEndToEndTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieKinematicsStreamingToolboxEndToEndTest extends KinematicsStreamingToolboxEndToEndTest
{
   private static final String RESOURCE_DIRECTORY = "us/ihmc/kinematicsStreamingToolboxLogs";

   @Override
   public DRCRobotModel newRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }

   @Tag("humanoid-toolbox")
   @Test
   public void testSimpleArmMotions() throws IOException
   {
      String fileName = "20190926_102802_ValkyrieKinematicsStreamingToolbox_SimpleArmMotions.json";
      InputStream inputStream = loadInputStream(fileName);
      runTest(inputStream);
   }

   @Tag("humanoid-toolbox")
   @Test
   public void testArmCollisions() throws IOException
   {
      String fileName = "20190926_101146_ValkyrieKinematicsStreamingToolbox_ArmCollisions.json";
      InputStream inputStream = loadInputStream(fileName);
      runTest(inputStream);
   }

   @Tag("humanoid-toolbox")
   @Test
   public void testCrazyInputsLog() throws IOException
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
