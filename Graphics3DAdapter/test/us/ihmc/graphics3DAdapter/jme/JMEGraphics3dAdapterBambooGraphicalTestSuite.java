package us.ihmc.graphics3DAdapter.jme;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.graphics3DAdapter.jme.lidar.JMEGPULidarTest.class,
   us.ihmc.graphics3DAdapter.jme.JMEGraphics3DWorldTest.class,
   us.ihmc.graphics3DAdapter.jme.JMERayCollisionAdapterTest.class,
   us.ihmc.graphics3DAdapter.jme.JMERendererTest.class,
   us.ihmc.graphics3DAdapter.jme.JMEMeshDataInterpreterTest.class
})

public class JMEGraphics3dAdapterBambooGraphicalTestSuite
{
   public static void main(String[] args)
   {
   }
}
