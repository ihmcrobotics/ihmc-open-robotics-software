package us.ihmc.atlas.parameters;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class AtlasSensorInformationTest
{
   @Test
   public void testSendRobotDataToROSIsFalse()
   {
      assertFalse("Do not check in SEND_ROBOT_DATA_TO_ROS = true!!", AtlasSensorInformation.SEND_ROBOT_DATA_TO_ROS);
   }
}
