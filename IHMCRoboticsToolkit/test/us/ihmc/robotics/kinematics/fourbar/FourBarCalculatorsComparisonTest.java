package us.ihmc.robotics.kinematics.fourbar;

import org.junit.Before;

import us.ihmc.tools.MemoryTools;


/**
 * It makes sure that the fast runner four bar calculator and the calculator used in Beast, Steppr, and Wanderer give the same result
 */
public class FourBarCalculatorsComparisonTest
{

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
  
}
