package us.ihmc.zulu.communication.producers;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBufferTest;

@Disabled
public class ZuluRobotConfigurationDataBufferTest extends RobotConfigurationDataBufferTest
{

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS).createFullRobotModel();
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