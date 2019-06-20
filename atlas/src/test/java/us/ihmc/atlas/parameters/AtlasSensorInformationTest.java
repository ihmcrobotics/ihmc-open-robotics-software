package us.ihmc.atlas.parameters;

import static us.ihmc.robotics.Assert.assertFalse;

import org.junit.jupiter.api.Test;
public class AtlasSensorInformationTest
{
   @Test
   public void testSendRobotDataToROSIsFalse()
   {
      assertFalse("Do not check in SEND_ROBOT_DATA_TO_ROS = true!!", AtlasSensorInformation.SEND_ROBOT_DATA_TO_ROS);
   }
}
