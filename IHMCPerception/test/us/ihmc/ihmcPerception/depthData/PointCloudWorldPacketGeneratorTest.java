package us.ihmc.ihmcPerception.depthData;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class PointCloudWorldPacketGeneratorTest
{

	@DeployableTestMethod(duration = 0.7)
   @Test(timeout = 30000)
   public void testGeneratePointCloudWorldPacket() throws InterruptedException
   {
      DepthDataFilter depthDataFilter = new DepthDataFilter();
      PointCloudWorldPacketGenerator generator = new PointCloudWorldPacketGenerator(depthDataFilter);

      Point3d sensorOrigin = new Point3d(0, 0, 1.0);
      for (double x = -10; x < 10; x += 0.01)
      {
         for (double y = -10; y < 10; y += 0.01)
         {
            Point3d point = new Point3d(x, y, Math.max(x + y, 0.0));
            depthDataFilter.addPoint(point, sensorOrigin);
         }
      }

      PointCloudWorldPacket packet = generator.getPointCloudWorldPacket();

      assertTrue(packet.getGroundQuadTreeSupport().length > 0);
      assertTrue(packet.getDecayingWorldScan().length > 0);

   }

}
