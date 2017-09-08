package us.ihmc.ihmcPerception.depthData;

import static org.junit.Assert.assertTrue;

import us.ihmc.euclid.tuple3D.Point3D;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;

public class PointCloudWorldPacketGeneratorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testGeneratePointCloudWorldPacket() throws InterruptedException
   {
      DepthDataFilter depthDataFilter = new DepthDataFilter();
      PointCloudWorldPacketGenerator generator = new PointCloudWorldPacketGenerator(depthDataFilter);

      Point3D sensorOrigin = new Point3D(0, 0, 1.0);
      for (double x = -10; x < 10; x += 0.01)
      {
         for (double y = -10; y < 10; y += 0.01)
         {
            Point3D point = new Point3D(x, y, Math.max(x + y, 0.0));
            depthDataFilter.addPoint(point, sensorOrigin);
         }
      }

      PointCloudWorldPacket packet = generator.getPointCloudWorldPacket();

      assertTrue(packet.getGroundQuadTreeSupport().length > 0);
      assertTrue(packet.getDecayingWorldScan().length > 0);

   }

}
