package us.ihmc.jMonkeyEngineToolkit;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.jMonkeyEngineToolkit.Graphics3DWorldTest.class,
   us.ihmc.jMonkeyEngineToolkit.Graphics3DTextTest.class,
   us.ihmc.jMonkeyEngineToolkit.utils.GraphicsDemoToolsTest.class
})

public class Graphics3DAdapterBambooJMETestSuite
{
   public static void main(String[] args)
   {
   }
}