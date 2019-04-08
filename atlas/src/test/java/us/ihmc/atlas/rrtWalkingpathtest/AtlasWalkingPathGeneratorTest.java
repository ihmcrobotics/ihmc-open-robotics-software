package us.ihmc.atlas.rrtWalkingpathtest;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasWalkingPathGeneratorTest extends AvatarWalkingPathGeneratorTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return "atlas";
   }

   @Override
   @Test
   public void testOne() throws SimulationExceededMaximumTimeException
   {
      super.testOne();
   }
}
