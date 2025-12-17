package us.ihmc.openAlexander.rrtWalkingPathTest;

import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.rrtWalkingPathTest.AvatarWalkingPathGeneratorTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AlexanderWalkingPathGeneratorTest extends AvatarWalkingPathGeneratorTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
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
