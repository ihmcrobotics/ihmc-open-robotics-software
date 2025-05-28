package us.ihmc.alexander.communication.producers;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.alexander.OpenAlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBufferTest;

@Disabled
public class AlexanderRobotConfigurationDataBufferTest extends RobotConfigurationDataBufferTest
{

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return new OpenAlexanderRobotModel(OpenAlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS).createFullRobotModel();
   }

   @Override
   @Test
   public void testAddingStuff()
   {
      super.testAddingStuff();
   }

   @Override
   @Test
   public void testWaitForTimestamp()
   {
      super.testWaitForTimestamp();
   }
}