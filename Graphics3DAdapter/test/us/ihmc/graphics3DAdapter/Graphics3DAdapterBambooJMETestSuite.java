package us.ihmc.graphics3DAdapter;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.JUnitTestSuiteGenerator;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.graphics3DAdapter.Graphics3DWorldTest.class,
   us.ihmc.graphics3DAdapter.Graphics3DTextTest.class,
   us.ihmc.graphics3DAdapter.Graphics3DObjectTest.class,
   us.ihmc.graphics3DAdapter.utils.GraphicsDemoToolsTest.class
})

public class Graphics3DAdapterBambooJMETestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteGenerator.generateTestSuite(Graphics3DAdapterBambooJMETestSuite.class);
   }
}
