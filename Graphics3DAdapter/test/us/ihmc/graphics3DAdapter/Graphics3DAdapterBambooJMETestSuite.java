package us.ihmc.graphics3DAdapter;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.graphics3DAdapter.Graphics3DWorldTest.class,
   us.ihmc.graphics3DAdapter.Graphics3DTextTest.class,
   us.ihmc.graphics3DAdapter.graphics.Graphics3DObjectTest.class,
   us.ihmc.graphics3DAdapter.utils.GraphicsDemoToolsTest.class
})

public class Graphics3DAdapterBambooJMETestSuite
{
   public static void main(String[] args)
   {
   }
}