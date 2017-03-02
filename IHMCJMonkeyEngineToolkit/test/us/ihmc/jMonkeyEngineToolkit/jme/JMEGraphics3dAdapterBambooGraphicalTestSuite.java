package us.ihmc.jMonkeyEngineToolkit.jme;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.jMonkeyEngineToolkit.jme.lidar.JMEGPULidarTest.class,
   us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DWorldTest.class,
   us.ihmc.jMonkeyEngineToolkit.jme.JMERayCollisionAdapterTest.class,
   us.ihmc.jMonkeyEngineToolkit.jme.JMERendererTest.class,
   us.ihmc.jMonkeyEngineToolkit.jme.JMEMeshDataInterpreterTest.class
})

public class JMEGraphics3dAdapterBambooGraphicalTestSuite
{
   public static void main(String[] args)
   {
   }
}
