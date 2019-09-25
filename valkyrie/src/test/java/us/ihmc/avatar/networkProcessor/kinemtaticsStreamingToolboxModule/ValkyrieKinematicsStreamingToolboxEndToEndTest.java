package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxEndToEndTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

import java.io.File;
import java.io.IOException;

public class ValkyrieKinematicsStreamingToolboxEndToEndTest extends KinematicsStreamingToolboxEndToEndTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false);
   }

   @Test
   public void testMessageReplay() throws IOException, SimulationExceededMaximumTimeException
   {
      runTest(new File("20190925_144316_ValkyrieKinematicsStreamingToolbox.json"));
   }
}
