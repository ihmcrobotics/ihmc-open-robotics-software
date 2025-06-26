package us.ihmc.openAlexander.networkProcessor.kinematicsStreamingToolboxModule;

import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.OpenAlexanderVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxControllerTest;

public class AlexanderKinematicsStreamingToolboxControllerTest extends KinematicsStreamingToolboxControllerTest
{
   @Override
   public DRCRobotModel newRobotModel()
   {
      return new OpenAlexanderRobotModel(OpenAlexanderVersion.V0_FULL_ROBOT);
   }

   @Override
   @Test
   public void testStreamingToController()
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
