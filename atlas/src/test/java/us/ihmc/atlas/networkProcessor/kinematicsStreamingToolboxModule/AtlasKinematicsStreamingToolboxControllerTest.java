package us.ihmc.atlas.networkProcessor.kinematicsStreamingToolboxModule;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxControllerTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasKinematicsStreamingToolboxControllerTest extends KinematicsStreamingToolboxControllerTest
{
   @Override
   public DRCRobotModel newRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
   }

   @Override
   @Test
   public void testStreamingToController() throws SimulationExceededMaximumTimeException
   {
      super.testStreamingToController();
   }

   @Override
   @Test
   public void testHandMotionWithCollision()
   {
      super.testHandMotionWithCollision();
   }
}
