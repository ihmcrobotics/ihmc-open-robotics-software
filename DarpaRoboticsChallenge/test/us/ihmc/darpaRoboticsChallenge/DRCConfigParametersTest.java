package us.ihmc.darpaRoboticsChallenge;

import static org.junit.Assert.assertFalse;

import org.junit.Test;

public class DRCConfigParametersTest
{

   @Test
   public void test()
   {
      assertFalse("Do not check in DRCConfigParameters.USE_PERFECT_SENSORS = true!!", DRCConfigParameters.USE_PERFECT_SENSORS);
   }

}
