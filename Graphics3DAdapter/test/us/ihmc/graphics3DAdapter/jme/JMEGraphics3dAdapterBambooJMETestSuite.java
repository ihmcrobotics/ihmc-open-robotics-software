package us.ihmc.graphics3DAdapter.jme;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.graphics3DAdapter.jme.lidar.JMEGPULidarTest.class,
   us.ihmc.graphics3DAdapter.jme.JMEGraphics3DWorldTest.class,
   us.ihmc.graphics3DAdapter.jme.JMERayCollisionAdapterTest.class,
   us.ihmc.graphics3DAdapter.jme.JMERendererTest.class
})

public class JMEGraphics3dAdapterBambooJMETestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteConstructor.generateTestSuite(JMEGraphics3dAdapterBambooJMETestSuite.class);
   }
}
