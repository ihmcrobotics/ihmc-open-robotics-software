package us.ihmc.zulu.rrtWalkingPathTest;

import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.rrtWalkingPathTest.AvatarWalkingPathGeneratorTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class ZuluWalkingPathGeneratorTest extends AvatarWalkingPathGeneratorTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return "alexander";
   }

   @Override
   @Test
   public void testOne() throws SimulationExceededMaximumTimeException
   {
      super.testOne();
   }
}
