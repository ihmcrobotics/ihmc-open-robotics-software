package us.ihmc.commonWalkingControlModules.controlModules;

import org.junit.After;
import org.junit.Before;

import us.ihmc.tools.MemoryTools;


public class GeometricVirtualToePointCalculatorLogisticParametersTest extends GeometricVirtualToePointCalculatorTest
{
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   protected void setParametersForGeometricVirtualToePointCalculator(GeometricVirtualToePointCalculator geometricVirtualToePointCalculator)
   {
      geometricVirtualToePointCalculator.setNewR2Parameters();
   }

}
