package us.ihmc.atlas.angularMomentumTest;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.angularMomentumTest.AvatarAngularExcursionTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@Tag("humanoid-flat-ground-slow-2")
public class AtlasAngularExcursionTest extends AvatarAngularExcursionTest
{
   private final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private final RobotTarget target = RobotTarget.SCS;

   @Override
   protected double getStepLength()
   {
      return 0.4;
   }

   @Override
   protected double getStepWidth()
   {
      return 0.25;
   }

   @Override
   @Test
   public void testWalkInASquare() throws SimulationExceededMaximumTimeException
   {
      super.testWalkInASquare();
   }


   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(version, target, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return "atlas";
   }
}
