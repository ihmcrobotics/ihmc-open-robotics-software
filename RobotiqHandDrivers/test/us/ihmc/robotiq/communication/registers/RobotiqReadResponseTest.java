package us.ihmc.robotiq.communication.registers;

import java.util.Random;

import org.junit.Test;

import us.ihmc.utilities.code.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.code.agileTesting.BambooPlanType;

@BambooPlan(planType = BambooPlanType.InDevelopment)
public class RobotiqReadResponseTest
{
   @Test(timeout = 3000)
   @EstimatedDuration(duration = 1000)
   public void testSetAll()
   {
      Random random = new Random(4270L);
      
   }
}
